"""Classes in this file are used to store raw or manipulated data about Mecademic robots.

Correct usage would be to build a RobotTrajectories object, and give it to ZipFileLogger.create_and_zip_files() to store
data in a file.

RobotTrajectories objects can then be reconstructed from those files using ZipFileLogger.unzip_and_open_files().

Raw data should be stored in a RobotTrajectories object using only robot_df_hist.output_dfs attribute (for data) and
robot_context.robot_information and robot_context.sent_commands (for info on how data was logged)

Manipulated or processed data will use all attributes of RobotTrajectories, storing intermediate data in
robot_df_hist.mid_dfs, statistics in robot_context.test_results and info on how statistics where produced in
robot_context.test_context
"""
from __future__ import annotations

import shutil
from dataclasses import dataclass, field
from pathlib import Path, PurePath
from tempfile import TemporaryDirectory

import pandas as pd
from dataclasses_json import dataclass_json


@dataclass_json
@dataclass
class TestContext:
    """ Context in which robot trajectory logs where produced, specifically those made by tests, and in which test
    were produced

    Attributes
    ----------
    testing_function_name: string
        Function or library in which test statistics and modified robot data was produced
    data_origin_files: list of strings
        Indicates file or files used to produced manipulated robot data and test results, or statistics
        Can either one or two files
    data_columns_inspected: list of list of strings
        Indicates columns in dataframes used to produce results
        Many options for data_columns_inspected:

        -A function could inspect many columns, one after the other: [['col1', 'col2', 'col3']]
        -A function could compare two columns: [['col1'],['col2']]
        -A function could compare many columns, one after the other:
         [['tg_col1', 'tg_col2', 'tg_col3'], ['col1', 'col2', 'col3']] (tg meaning target)
    """
    testing_function_name: str = field(default='')
    data_origin_files: list[str] = field(default_factory=list)
    data_columns_inspected: list[list[str]] = field(default_factory=list)


@dataclass_json
@dataclass
class RobotContext:
    """ Context in which robot trajectory logs where produced

    Attributes
    ----------
    robot_information: list of dicts
        This list, when produced by the logger, will contain one dict filled with information about a robot. Could
        contain more dicts after test on robot rt_data from many robots
    sent_commands: list or strings
        This list, produced by the logger, should contain the commands from which the robot movement was produced.
        Tests should only compare robot rt_data that came from same set of commands
    test_context: dict
        This dict should contain context on tests made on robot rt_data after data was logged. This will not
        be filled by the logger
    test_results: dict
        This dict should contain results of tests made on robot rt_data after data was logged. This will not
        be filled by the logger.
    """
    robot_information: list[dict[str, str]] = field(default_factory=list)
    sent_commands: list[str] = field(default_factory=list)
    test_context: TestContext = field(default_factory=TestContext)
    test_results: dict[str, dict[str, str]] = field(default_factory=dict)

    def to_file(self, filename):
        """ Creates a json file in which is stored relevant context

        Parameters
        ----------
        robot_context: Robot_Context object
            Context to store in json file
        filename: string
            Name given to the file in which info is stored. '.json' is added here
        """

        # Note: to_json comes from @dataclass_json
        # pylint: disable=no-member
        context_json = self.to_json(indent=4)

        with open(f'{filename}.json', 'w', encoding='utf-8') as file:
            file.write(context_json)

    @staticmethod
    def from_file(filepath):
        """ Finds robot context in a json file

        Parameters
        ----------
        filepath: string
            Complete path to the relevant file, including its name, but excluding '.json' extension

        Returns
        -------
        robot_context: RobotContext object
            Context info built from json file
        """

        with open(filepath, encoding='utf-8') as file:
            file_content = file.read()
            # Note: from_json comes from @dataclass_json
            # pylint: disable=no-member
            robot_context = RobotContext.from_json(file_content)

        return robot_context


@dataclass
class RobotDfHist:
    """This class contains all robot rt_data dataframes produced by a function, whether they were produced at the
    beginning, during or at the end of the function.

    Attributes
    ----------
    input_dfs: list of Pandas dataframes
        Dataframes from which all processing is made
    mid_dfs: dict of Pandas Dataframes
        Intermediate results of processing
    output_dfs: list of dataframes
        Final results of processing. 'RobotDfHist' should contain at least one dataframe in this list (this is the only
        attribute that is needed at the end of a function producing a 'RobotDfHist', all other attributes are optional)
    """
    input_dfs: list[pd.DataFrame] = field(default_factory=list)
    mid_dfs: dict[str, pd.DataFrame] = field(default_factory=dict)
    output_dfs: list[pd.DataFrame] = field(default_factory=list)

    def from_other(self, other, input_names=None, output_names=None):
        """This function fills this object with the contents of another 'RobotDfHist'

        Parameters
        ----------
        other : RobotDfHist object
            Contains dataframes to store in this object
        input_names : list of strings, optional
            If this list contains strings, it will put dataframes from input_dfs of other in the mid_dfs dict of this
            object, associating them to the strings in 'input_names'
        output_names : list of strings, optional
            If this list contains strings, it will put dataframes from 'output_dfs' of other in the 'mid_dfs' dict of
            this object, associating them to the strings in 'output_names'. Otherwise, the contents of
            'other.output_dfs' will be put into this object's 'output_dfs'
        """
        if input_names:
            self.add_list_to_intermediate(other.input_dfs, input_names)

        if output_names:
            self.add_list_to_intermediate(other.output_dfs, output_names)
        else:
            self.output_dfs = other.output_dfs

        self.mid_dfs.update(other.mid_dfs)

    def add_list_to_intermediate(self, dfs, names):
        """Adds some dataframes to 'mid_dfs'

        Parameters
        ----------
        dfs : list of Pandas dataframes
            Dataframes to add
        names : list of strings
            Names to associate to each df in 'dfs'
        """
        for df, name in zip(dfs, names):
            self.mid_dfs[name] = df

    def make_dict(self):
        """Creates a dictionary using all dataframes found in this object

        Returns
        -------
        dict of Pandas dataframes
            Contains all dataframes in 'input_dfs', 'mid_dfs', 'output_dfs'

        Returns
        -------
        out_dict: dict of Pandas dataframes
            Dataframes stored in all attributes of this object, with special names from dataframes from input_dfs and
            output_dfs
        """
        out_dict = dict()
        out_dict.update(self.mid_dfs)
        if len(self.input_dfs) == 1:
            out_dict[self.base_input_name()] = self.input_dfs[0]
        else:
            for df, i in zip(self.input_dfs, range(0, len(self.input_dfs))):
                out_dict[self.base_input_name() + '_' + str(i)] = df

        if len(self.output_dfs) == 1:
            out_dict[self.base_output_name()] = self.output_dfs[0]
        else:
            for df, i in zip(self.output_dfs, range(0, len(self.output_dfs))):
                out_dict[self.base_output_name() + '_' + str(i)] = df

        return out_dict

    def build_from_dict(self, df_dict):
        """Recreates a 'RobotDfHist' object from a dict

        Parameters
        ----------
        df_dict : dict of Pandas dataframes
            Produced by method 'make_dict'
        """

        for key, df in df_dict.items():
            result, self.input_dfs = self.insert_in_list_from_dict(key, df, self.input_dfs, self.base_input_name)
            result, self.output_dfs = self.insert_in_list_from_dict(key, df, self.output_dfs, self.base_output_name)
            if not result:
                self.mid_dfs[key] = df

    def insert_in_list_from_dict(self, key, df, attr_list, list_prefix_func):
        """Identifies to which attribute a dataframe coming from a dict made by 'make_dict' belongs to and adds it to
        this attribute

        Parameters
        ----------
        key : string
            Used to identify to which attribute 'df' belongs to
        df : Pandas dataframe
            To add to an attribute
        attr_list : list of Pandas dataframe
            Attribute to add the dataframe to if it belongs to it
        list_prefix_func : function
            function used to identify if 'key, and thus 'df', belongs to 'attr_list'

        Returns
        -------
        list of Pandas dataframe
            updated attribute list, containing 'df' if 'df' belonged in 'attr_list'
        """
        result = True
        if key == list_prefix_func():
            attr_list = [df]
        elif key.startswith(list_prefix_func()):
            index = int(key[-1])
            try:
                attr_list[index] = df
            except IndexError:
                for _ in range(index - len(attr_list) + 1):
                    attr_list.append(None)
                attr_list[index] = df
        else:
            result = False

        return result, attr_list  # It seems if we pass attribute as argument, even if attribute is passed by reference,
        # it won't update the attribute, so we have to return the list

    def base_input_name(self):
        """Returns key prefix used in 'make_dict' for 'input_dfs'
        """
        return 'input_df'

    def base_output_name(self):
        """Returns key prefix used in 'make_dict' for 'output_dfs'
        """
        return 'output_df'

    def __eq__(self, other) -> bool:
        """Returns true if both RobotDfHist objects contain same dataframes, in same position of each attribute

        Parameters
        ----------
        other : RobotDfHist object
            Object compared to self

        Returns
        -------
        bool
            True if all dataframes in both objects are the same
        """
        for index, df in enumerate(self.input_dfs):
            if not df.equals(other.input_dfs[index]):
                return False
        for key, df in self.mid_dfs.items():
            if not df.equals(other.mid_dfs[key]):
                return False
        for index, df in enumerate(self.output_dfs):
            if not df.equals(other.output_dfs[index]):
                return False
        return True

    @staticmethod
    def to_file(df, filename):
        """ Creates a csv file in which is stored relevant data

        Parameters
        ----------
        df: Pandas dataframe
            Data to store in csv file
        filename: string
            Name given to the file in which info is stored. '.csv' is added here
        """
        df.to_csv(f'{filename}.csv', index_label='timestamp')

    @staticmethod
    def from_file(filepath):
        """ Finds robot rt_data in a csv file

        Parameters
        ----------
        filepath: string
            Complete path to the relevant file, including its name

        Returns
        -------
        df: Pandas dataframe
            Data stored in csv file
        """

        df = pd.read_csv(filepath, index_col='timestamp')

        return df


@dataclass
class RobotTrajectories:
    """Robot movement through time and context in which rt_data was produced

    Attributes
    ----------
    robot_context: RobotContext object
        Explains how and where dfs where produced
    robot_df_hist: RobotDfHist object
        This object, when produced by the logger, should contain only one dataframe, in the output_dfs list, associating
        timestamps to robot rt_data through time

        All other dataframes in this object could be produced by subsequent tests and manipulations made on the data
        from the first dataframe
    """
    robot_context: RobotContext = field(default_factory=RobotContext)
    robot_df_hist: RobotDfHist = field(default_factory=RobotDfHist)

    def to_file(self, filename, file_path=None):
        """ Creates a zipped directory in which robot context and associated data is stored in many files

        Parameters
        ----------
        robot_trajectories: RobotTrajectories object
            Data and context to store in zipped directory, in many files
        filename: string
            Name given to the zipped directory in which info is stored. '.zip' is added here
        filepath: string
            Directory in which to save zipped file
        """

        current_dir = Path.cwd()

        with TemporaryDirectory(dir=current_dir) as root_dir:

            self.robot_context.to_file(PurePath.joinpath(PurePath(root_dir), filename))

            for key, df in self.robot_df_hist.make_dict().items():
                RobotDfHist.to_file(df, PurePath.joinpath(PurePath(root_dir), '_'.join([filename, key])))

            shutil.make_archive(base_name=filename, format='zip', root_dir=root_dir)

        if file_path:
            shutil.move(
                PurePath.joinpath(current_dir, filename + '.zip'),
                PurePath.joinpath(PurePath(file_path), filename + '.zip'),
            )

    @staticmethod
    def from_file(filepath):
        """ Finds robot context and rt_data data in a zip file

        Parameters
        ----------
        filepath: string
            Complete path to the relevant zipped file, including its name, with the '.zip' extension

        Returns
        -------
        robot_trajectories: RobotTrajectories object
            Trajectory object built from content of zipped file
        """
        current_dir = Path.cwd()

        with TemporaryDirectory(dir=current_dir) as base_dir:

            shutil.unpack_archive(filepath, extract_dir=base_dir)

            robot_trajectories = RobotTrajectories()

            base_name = PurePath(filepath).stem  # Keep file name without extension

            files = Path(base_dir).glob('*')  # .glob from Path

            df_dict = dict()

            for file in files:
                if file.name.endswith('.csv'):
                    base_name_file = PurePath(file.name).stem  # Keep file name without extension
                    base_name_file = base_name_file.removeprefix(base_name + '_')
                    df_dict[base_name_file] = RobotDfHist.from_file(PurePath.joinpath(PurePath(base_dir), file))
                elif file.name.endswith('.json'):
                    robot_trajectories.robot_context = RobotContext.from_file(
                        PurePath.joinpath(PurePath(base_dir), file))
                else:
                    raise ValueError(
                        "Unsupported extension. Cannot open and parse this file to retrieve RobotTrajectories")

            robot_trajectories.robot_df_hist.build_from_dict(df_dict)

        return robot_trajectories
