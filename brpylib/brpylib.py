# -*- coding: utf-8 -*-
"""
Collection of classes used for reading headers and data from Blackrock files
current version: 2.0.1 --- 11/12/2021

@author: Mitch Frankel - Blackrock Microsystems
     Stephen Hou - v1.4.0 edits
     David Kluger - v2.0.0 overhaul

Version History:
v1.0.0 - 07/05/2016 - initial release - requires brMiscFxns v1.0.0
v1.1.0 - 07/08/2016 - inclusion of NsxFile.savesubsetnsx() for saving subset of Nsx data to disk4
v1.1.1 - 07/09/2016 - update to NsxFile.savesubsetnsx() for option (not)overwriting subset files if already exist
                      bug fixes in NsxFile class as reported from beta user
v1.2.0 - 07/12/2016 - bug fixes in NsxFile.savesubsetnsx()
                      added version control and checking for brMiscFxns
                      requires brMiscFxns v1.1.0
v1.3.0 - 07/22/2016 - added 'samp_per_s' to NsxFile.getdata() output
                      added close() method to NsxFile and NevFile objects
                      NsxFile.getdata() now pre-allocates output['data'] as zeros - speed and safety
v1.3.1 - 08/02/2016 - bug fixes to NsxFile.getdata() for usability with Python 2.7 as reported from beta user
                      patch for use with multiple NSP sync (overwriting of initial null data from initial data packet)
                      __future__ import for use with Python 2.7 (division)
                      minor modifications to allow use of Python 2.6+
v1.3.2 - 08/12/2016 - bug fixes to NsXFile.getdata()
v1.4.0 - 06/22/2017 - inclusion of wave_read parameter to NevFile.getdata() for including/excluding waveform data
v2.0.0 - 04/27/2021 - numpy-based architecture rebuild of NevFile.getdata()
v2.0.1 - 11/12/2021 - fixed indexing error in NevFile.getdata()
                      Added numpy architecture to NsxFile.getdata()
"""


from __future__ import division  # for those using Python 2.6+

from collections import namedtuple
from datetime import datetime
from math import ceil
from os import path as ospath
from struct import calcsize, pack, unpack, unpack_from

import numpy as np

from .brMiscFxns import brmiscfxns_ver, openfilecheck

# Version control set/check
brpylib_ver = "2.0.1"
brmiscfxns_ver_req = "1.2.0"
if brmiscfxns_ver.split(".") < brmiscfxns_ver_req.split("."):
    raise Exception(
        "brpylib requires brMiscFxns "
        + brmiscfxns_ver_req
        + " or higher, please use latest version"
    )

# Patch for use with Python 2.6+
try:
    input = raw_input
except NameError:
    pass

# Define global variables to remove magic numbers
# <editor-fold desc="Globals">
WARNING_SLEEP_TIME = 5
DATA_PAGING_SIZE = 1024**3
DATA_FILE_SIZE_MIN = 1024**2 * 10
STRING_TERMINUS = "\x00"
UNDEFINED = 0
ELEC_ID_DEF = "all"
START_TIME_DEF = 0
DATA_TIME_DEF = "all"
DOWNSAMPLE_DEF = 1
START_OFFSET_MIN = 0
STOP_OFFSET_MIN = 0

UV_PER_BIT_21 = 0.25
WAVEFORM_SAMPLES_21 = 48
NSX_BASIC_HEADER_BYTES_22 = 314
NSX_EXT_HEADER_BYTES_22 = 66
DATA_BYTE_SIZE = 2
TIMESTAMP_NULL_21 = 0

NO_FILTER = 0
BUTTER_FILTER = 1
SERIAL_MODE = 0

RB2D_MARKER = 1
RB2D_BLOB = 2
RB3D_MARKER = 3
BOUNDARY_2D = 4
MARKER_SIZE = 5

DIGITAL_PACKET_ID = 0
NEURAL_PACKET_ID_MIN = 1
NEURAL_PACKET_ID_MAX = 16384
COMMENT_PACKET_ID = 65535
VIDEO_SYNC_PACKET_ID = 65534
TRACKING_PACKET_ID = 65533
BUTTON_PACKET_ID = 65532
CONFIGURATION_PACKET_ID = 65531

PARALLEL_REASON = 1
PERIODIC_REASON = 64
SERIAL_REASON = 129
LOWER_BYTE_MASK = 255
FIRST_BIT_MASK = 1
SECOND_BIT_MASK = 2

CLASSIFIER_MIN = 1
CLASSIFIER_MAX = 16
CLASSIFIER_NOISE = 255

CHARSET_ANSI = 0
CHARSET_UTF = 1
CHARSET_ROI = 255

COMM_RGBA = 0
COMM_TIME = 1

BUTTON_PRESS = 1
BUTTON_RESET = 2

CHG_NORMAL = 0
CHG_CRITICAL = 1

ENTER_EVENT = 1
EXIT_EVENT = 2
# </editor-fold>

# Define a named tuple that has information about header/packet fields
FieldDef = namedtuple("FieldDef", ["name", "formatStr", "formatFnc"])


# <editor-fold desc="Header processing functions">
def processheaders(curr_file, packet_fields):
    """
    :param curr_file:      {file} the current BR datafile to be processed
    :param packet_fields : {named tuple} the specific binary fields for the given header
    :return:               a fully unpacked and formatted tuple set of header information

    Read a packet from a binary data file and return a list of fields
    The amount and format of data read will be specified by the
    packet_fields container
    """

    # This is a lot in one line.  First I pull out all the format strings from
    # the basic_header_fields named tuple, then concatenate them into a string
    # with '<' at the front (for little endian format)
    packet_format_str = "<" + "".join([fmt for name, fmt, fun in packet_fields])

    # Calculate how many bytes to read based on the format strings of the header fields
    bytes_in_packet = calcsize(packet_format_str)
    packet_binary = curr_file.read(bytes_in_packet)

    # unpack the binary data from the header based on the format strings of each field.
    # This returns a list of data, but it's not always correctly formatted (eg, FileSpec
    # is read as ints 2 and 3 but I want it as '2.3'
    packet_unpacked = unpack(packet_format_str, packet_binary)

    # Create a iterator from the data list.  This allows a formatting function
    # to use more than one item from the list if needed, and the next formatting
    # function can pick up on the correct item in the list
    data_iter = iter(packet_unpacked)

    # create an empty dictionary from the name field of the packet_fields.
    # The loop below will fill in the values with formatted data by calling
    # each field's formatting function
    packet_formatted = dict.fromkeys([name for name, fmt, fun in packet_fields])
    for name, fmt, fun in packet_fields:
        packet_formatted[name] = fun(data_iter)

    return packet_formatted


def format_filespec(header_list):
    return str(next(header_list)) + "." + str(next(header_list))  # eg 2.3


def format_timeorigin(header_list):
    year = next(header_list)
    month = next(header_list)
    _ = next(header_list)
    day = next(header_list)
    hour = next(header_list)
    minute = next(header_list)
    second = next(header_list)
    millisecond = next(header_list)
    return datetime(year, month, day, hour, minute, second, millisecond * 1000)


def format_stripstring(header_list):
    string = bytes.decode(next(header_list), "latin-1")
    return string.split(STRING_TERMINUS, 1)[0]


def format_none(header_list):
    return next(header_list)


def format_freq(header_list):
    return str(float(next(header_list)) / 1000) + " Hz"


def format_filter(header_list):
    filter_type = next(header_list)
    if filter_type == NO_FILTER:
        return "none"
    elif filter_type == BUTTER_FILTER:
        return "butterworth"


def format_charstring(header_list):
    return int(next(header_list))


def format_digconfig(header_list):
    config = next(header_list) & FIRST_BIT_MASK
    if config:
        return "active"
    else:
        return "ignored"


def format_anaconfig(header_list):
    config = next(header_list)
    if config & FIRST_BIT_MASK:
        return "low_to_high"
    if config & SECOND_BIT_MASK:
        return "high_to_low"
    else:
        return "none"


def format_digmode(header_list):
    dig_mode = next(header_list)
    if dig_mode == SERIAL_MODE:
        return "serial"
    else:
        return "parallel"


def format_trackobjtype(header_list):
    trackobj_type = next(header_list)
    if trackobj_type == UNDEFINED:
        return "undefined"
    elif trackobj_type == RB2D_MARKER:
        return "2D RB markers"
    elif trackobj_type == RB2D_BLOB:
        return "2D RB blob"
    elif trackobj_type == RB3D_MARKER:
        return "3D RB markers"
    elif trackobj_type == BOUNDARY_2D:
        return "2D boundary"
    elif trackobj_type == MARKER_SIZE:
        return "marker size"
    else:
        return "error"


def getdigfactor(ext_headers, idx):
    max_analog = ext_headers[idx]["MaxAnalogValue"]
    min_analog = ext_headers[idx]["MinAnalogValue"]
    max_digital = ext_headers[idx]["MaxDigitalValue"]
    min_digital = ext_headers[idx]["MinDigitalValue"]
    return float(max_analog - min_analog) / float(max_digital - min_digital)


# </editor-fold>


# <editor-fold desc="Header dictionaries">
nev_header_dict = {
    "basic": [
        FieldDef("FileTypeID", "8s", format_stripstring),  # 8 bytes   - 8 char array
        FieldDef("FileSpec", "2B", format_filespec),  # 2 bytes   - 2 unsigned char
        FieldDef("AddFlags", "H", format_none),  # 2 bytes   - uint16
        FieldDef("BytesInHeader", "I", format_none),  # 4 bytes   - uint32
        FieldDef("BytesInDataPackets", "I", format_none),  # 4 bytes   - uint32
        FieldDef("TimeStampResolution", "I", format_none),  # 4 bytes   - uint32
        FieldDef("SampleTimeResolution", "I", format_none),  # 4 bytes   - uint32
        FieldDef("TimeOrigin", "8H", format_timeorigin),  # 16 bytes  - 8 x uint16
        FieldDef(
            "CreatingApplication", "32s", format_stripstring
        ),  # 32 bytes  - 32 char array
        FieldDef("Comment", "256s", format_stripstring),  # 256 bytes - 256 char array
        FieldDef("NumExtendedHeaders", "I", format_none),
    ],  # 4 bytes   - uint32
    "ARRAYNME": FieldDef(
        "ArrayName", "24s", format_stripstring
    ),  # 24 bytes  - 24 char array
    "ECOMMENT": FieldDef(
        "ExtraComment", "24s", format_stripstring
    ),  # 24 bytes  - 24 char array
    "CCOMMENT": FieldDef(
        "ContComment", "24s", format_stripstring
    ),  # 24 bytes  - 24 char array
    "MAPFILE": FieldDef(
        "MapFile", "24s", format_stripstring
    ),  # 24 bytes  - 24 char array
    "NEUEVWAV": [
        FieldDef("ElectrodeID", "H", format_none),  # 2 bytes  - uint16
        FieldDef(
            "PhysicalConnector", "B", format_charstring
        ),  # 1 byte   - 1 unsigned char
        FieldDef("ConnectorPin", "B", format_charstring),  # 1 byte   - 1 unsigned char
        FieldDef("DigitizationFactor", "H", format_none),  # 2 bytes  - uint16
        FieldDef("EnergyThreshold", "H", format_none),  # 2 bytes  - uint16
        FieldDef("HighThreshold", "h", format_none),  # 2 bytes  - int16
        FieldDef("LowThreshold", "h", format_none),  # 2 bytes  - int16
        FieldDef(
            "NumSortedUnits", "B", format_charstring
        ),  # 1 byte   - 1 unsigned char
        FieldDef(
            "BytesPerWaveform", "B", format_charstring
        ),  # 1 byte   - 1 unsigned char
        FieldDef("SpikeWidthSamples", "H", format_none),  # 2 bytes  - uint16
        FieldDef("EmptyBytes", "8s", format_none),
    ],  # 8 bytes  - empty
    "NEUEVLBL": [
        FieldDef("ElectrodeID", "H", format_none),  # 2 bytes  - uint16
        FieldDef("Label", "16s", format_stripstring),  # 16 bytes - 16 char array
        FieldDef("EmptyBytes", "6s", format_none),
    ],  # 6 bytes  - empty
    "NEUEVFLT": [
        FieldDef("ElectrodeID", "H", format_none),  # 2 bytes  - uint16
        FieldDef("HighFreqCorner", "I", format_freq),  # 4 bytes  - uint32
        FieldDef("HighFreqOrder", "I", format_none),  # 4 bytes  - uint32
        FieldDef("HighFreqType", "H", format_filter),  # 2 bytes  - uint16
        FieldDef("LowFreqCorner", "I", format_freq),  # 4 bytes  - uint32
        FieldDef("LowFreqOrder", "I", format_none),  # 4 bytes  - uint32
        FieldDef("LowFreqType", "H", format_filter),  # 2 bytes  - uint16
        FieldDef("EmptyBytes", "2s", format_none),
    ],  # 2 bytes  - empty
    "DIGLABEL": [
        FieldDef("Label", "16s", format_stripstring),  # 16 bytes - 16 char array
        FieldDef("Mode", "?", format_digmode),  # 1 byte   - boolean
        FieldDef("EmptyBytes", "7s", format_none),
    ],  # 7 bytes  - empty
    "NSASEXEV": [
        FieldDef("Frequency", "H", format_none),  # 2 bytes  - uint16
        FieldDef(
            "DigitalInputConfig", "B", format_digconfig
        ),  # 1 byte   - 1 unsigned char
        FieldDef(
            "AnalogCh1Config", "B", format_anaconfig
        ),  # 1 byte   - 1 unsigned char
        FieldDef("AnalogCh1DetectVal", "h", format_none),  # 2 bytes  - int16
        FieldDef(
            "AnalogCh2Config", "B", format_anaconfig
        ),  # 1 byte   - 1 unsigned char
        FieldDef("AnalogCh2DetectVal", "h", format_none),  # 2 bytes  - int16
        FieldDef(
            "AnalogCh3Config", "B", format_anaconfig
        ),  # 1 byte   - 1 unsigned char
        FieldDef("AnalogCh3DetectVal", "h", format_none),  # 2 bytes  - int16
        FieldDef(
            "AnalogCh4Config", "B", format_anaconfig
        ),  # 1 byte   - 1 unsigned char
        FieldDef("AnalogCh4DetectVal", "h", format_none),  # 2 bytes  - int16
        FieldDef(
            "AnalogCh5Config", "B", format_anaconfig
        ),  # 1 byte   - 1 unsigned char
        FieldDef("AnalogCh5DetectVal", "h", format_none),  # 2 bytes  - int16
        FieldDef("EmptyBytes", "6s", format_none),
    ],  # 2 bytes  - empty
    "VIDEOSYN": [
        FieldDef("VideoSourceID", "H", format_none),  # 2 bytes  - uint16
        FieldDef("VideoSource", "16s", format_stripstring),  # 16 bytes - 16 char array
        FieldDef("FrameRate", "f", format_none),  # 4 bytes  - single float
        FieldDef("EmptyBytes", "2s", format_none),
    ],  # 2 bytes  - empty
    "TRACKOBJ": [
        FieldDef("TrackableType", "H", format_trackobjtype),  # 2 bytes  - uint16
        FieldDef("TrackableID", "I", format_none),  # 4 bytes  - uint32
        # FieldDef('PointCount',         'H',    format_none),           # 2 bytes  - uint16
        FieldDef("VideoSource", "16s", format_stripstring),  # 16 bytes - 16 char array
        FieldDef("EmptyBytes", "2s", format_none),
    ],  # 2 bytes  - empty
}

nsx_header_dict = {
    "basic_21": [
        FieldDef("Label", "16s", format_stripstring),  # 16 bytes  - 16 char array
        FieldDef("Period", "I", format_none),  # 4 bytes   - uint32
        FieldDef("ChannelCount", "I", format_none),
    ],  # 4 bytes   - uint32
    "basic": [
        FieldDef("FileSpec", "2B", format_filespec),  # 2 bytes   - 2 unsigned char
        FieldDef("BytesInHeader", "I", format_none),  # 4 bytes   - uint32
        FieldDef("Label", "16s", format_stripstring),  # 16 bytes  - 16 char array
        FieldDef("Comment", "256s", format_stripstring),  # 256 bytes - 256 char array
        FieldDef("Period", "I", format_none),  # 4 bytes   - uint32
        FieldDef("TimeStampResolution", "I", format_none),  # 4 bytes   - uint32
        FieldDef("TimeOrigin", "8H", format_timeorigin),  # 16 bytes  - 8 uint16
        FieldDef("ChannelCount", "I", format_none),
    ],  # 4 bytes   - uint32
    "extended": [
        FieldDef("Type", "2s", format_stripstring),  # 2 bytes   - 2 char array
        FieldDef("ElectrodeID", "H", format_none),  # 2 bytes   - uint16
        FieldDef(
            "ElectrodeLabel", "16s", format_stripstring
        ),  # 16 bytes  - 16 char array
        FieldDef("PhysicalConnector", "B", format_none),  # 1 byte    - uint8
        FieldDef("ConnectorPin", "B", format_none),  # 1 byte    - uint8
        FieldDef("MinDigitalValue", "h", format_none),  # 2 bytes   - int16
        FieldDef("MaxDigitalValue", "h", format_none),  # 2 bytes   - int16
        FieldDef("MinAnalogValue", "h", format_none),  # 2 bytes   - int16
        FieldDef("MaxAnalogValue", "h", format_none),  # 2 bytes   - int16
        FieldDef("Units", "16s", format_stripstring),  # 16 bytes  - 16 char array
        FieldDef("HighFreqCorner", "I", format_freq),  # 4 bytes   - uint32
        FieldDef("HighFreqOrder", "I", format_none),  # 4 bytes   - uint32
        FieldDef("HighFreqType", "H", format_filter),  # 2 bytes   - uint16
        FieldDef("LowFreqCorner", "I", format_freq),  # 4 bytes   - uint32
        FieldDef("LowFreqOrder", "I", format_none),  # 4 bytes   - uint32
        FieldDef("LowFreqType", "H", format_filter),
    ],  # 2 bytes   - uint16
    "data": [
        FieldDef("Header", "B", format_none),  # 1 byte    - uint8
        FieldDef("Timestamp", "I", format_none),  # 4 bytes   - uint32
        FieldDef("NumDataPoints", "I", format_none),
    ],  # 4 bytes   - uint32]
}
# </editor-fold>


# <editor-fold desc="Safety check functions">
def check_elecid(elec_ids):
    if type(elec_ids) is str and elec_ids != ELEC_ID_DEF:
        print(
            "\n*** WARNING: Electrode IDs must be 'all', a single integer, or a list of integers."
        )
        print("      Setting elec_ids to 'all'")
        elec_ids = ELEC_ID_DEF
    if elec_ids != ELEC_ID_DEF and type(elec_ids) is not list:
        if type(elec_ids) == range:
            elec_ids = list(elec_ids)
        elif type(elec_ids) == int:
            elec_ids = [elec_ids]
    return elec_ids


def check_starttime(start_time_s):
    if not isinstance(start_time_s, (int, float)) or (
        isinstance(start_time_s, (int, float)) and start_time_s < START_TIME_DEF
    ):
        print("\n*** WARNING: Start time is not valid, setting start_time_s to 0")
        start_time_s = START_TIME_DEF
    return start_time_s


def check_datatime(data_time_s):
    if (type(data_time_s) is str and data_time_s != DATA_TIME_DEF) or (
        isinstance(data_time_s, (int, float)) and data_time_s < 0
    ):
        print("\n*** WARNING: Data time is not valid, setting data_time_s to 'all'")
        data_time_s = DATA_TIME_DEF
    return data_time_s


def check_downsample(downsample):
    if not isinstance(downsample, int) or downsample < DOWNSAMPLE_DEF:
        print(
            "\n*** WARNING: Downsample must be an integer value greater than 0. "
            "      Setting downsample to 1 (no downsampling)"
        )
        downsample = DOWNSAMPLE_DEF
    return downsample


def check_dataelecid(elec_ids, all_elec_ids):
    unique_elec_ids = set(elec_ids)
    all_elec_ids = set(all_elec_ids)

    # if some electrodes asked for don't exist, reset list with those that do, or throw error and return
    if not unique_elec_ids.issubset(all_elec_ids):
        if not unique_elec_ids & all_elec_ids:
            print("\nNone of the elec_ids passed exist in the data, returning None")
            return None
        else:
            print(
                "\n*** WARNING: Channels "
                + str(sorted(list(unique_elec_ids - all_elec_ids)))
                + " do not exist in the data"
            )
            unique_elec_ids = unique_elec_ids & all_elec_ids

    return sorted(list(unique_elec_ids))


def check_filesize(file_size):
    if file_size < DATA_FILE_SIZE_MIN:
        print("\n file_size must be larger than 10 Mb, setting file_size=10 Mb")
        return DATA_FILE_SIZE_MIN
    else:
        return int(file_size)


# </editor-fold>


class NevFile:
    """
    attributes and methods for all BR event data files.  Initialization opens the file and extracts the
    basic header information.
    """

    def __init__(self, datafile=""):
        self.datafile = datafile
        self.basic_header = {}
        self.extended_headers = []
        
        self.openfile()
        
    def openfile(self):
        # Run openfilecheck and open the file passed or allow user to browse to one
        self.datafile = openfilecheck(
            "rb",
            file_name=self.datafile,
            file_ext=".nev",
            file_type="Blackrock NEV Files",
        )

        # extract basic header information
        self.basic_header = processheaders(self.datafile, nev_header_dict["basic"])

        # Extract extended headers
        for i in range(self.basic_header["NumExtendedHeaders"]):
            self.extended_headers.append({})
            header_string = bytes.decode(
                unpack("<8s", self.datafile.read(8))[0], "latin-1"
            )
            self.extended_headers[i]["PacketID"] = header_string.split(
                STRING_TERMINUS, 1
            )[0]
            self.extended_headers[i].update(
                processheaders(
                    self.datafile, nev_header_dict[self.extended_headers[i]["PacketID"]]
                )
            )

            # Must set this for file spec 2.1 and 2.2
            if (
                header_string == "NEUEVWAV"
                and float(self.basic_header["FileSpec"]) < 2.3
            ):
                self.extended_headers[i]["SpikeWidthSamples"] = WAVEFORM_SAMPLES_21

    def getdata(self, elec_ids="all", wave_read="read"):
        """
        This function is used to return a set of data from the NEV datafile.

        :param elec_ids: [optional] {list} User selection of elec_ids to extract specific spike waveforms (e.g., [13])
        :param wave_read: [optional] {STR} 'read' or 'no_read' - whether to read waveforms or not
        :return: output: {Dictionary} with one or more of the following dictionaries (all include TimeStamps)
                    dig_events:            Reason, Data, [for file spec 2.2 and below, AnalogData and AnalogDataUnits]
                    spike_events:          Units='nV', ChannelID, NEUEVWAV_HeaderIndices, Classification, Waveforms
                    comments:              CharSet, Flag, Data, Comment
                    video_sync_events:     VideoFileNum, VideoFrameNum, VideoElapsedTime_ms, VideoSourceID
                    tracking_events:       ParentID, NodeID, NodeCount, TrackingPoints
                    button_trigger_events: TriggerType
                    configuration_events:  ConfigChangeType

        Note: For digital and neural data - TimeStamps, Classification, and Data can be lists of lists when more
        than one digital type or spike event exists for a channel
        """

        # Initialize output dictionary and reset position in file (if read before, may not be here anymore)
        output = dict()

        # Safety checks
        elec_ids = check_elecid(elec_ids)

        ######
        # extract raw data
        self.datafile.seek(0, 2)
        lData = self.datafile.tell()
        nPackets = int(
            (lData - self.basic_header["BytesInHeader"])
            / self.basic_header["BytesInDataPackets"]
        )
        self.datafile.seek(self.basic_header["BytesInHeader"], 0)
        rawdata = self.datafile.read()
        # rawdataArray = np.reshape(np.fromstring(rawdata,'B'),(nPackets,self.basic_header['BytesInDataPackets']))

        # Find all timestamps and PacketIDs
        if self.basic_header["FileTypeID"] == "BREVENTS":
            tsBytes = 8
            ts = np.ndarray(
                (nPackets,),
                "<L",
                rawdata,
                0,
                (self.basic_header["BytesInDataPackets"],),
            )
        else:
            tsBytes = 4
            ts = np.ndarray(
                (nPackets,),
                "<I",
                rawdata,
                0,
                (self.basic_header["BytesInDataPackets"],),
            )

        PacketID = np.ndarray(
            (nPackets,),
            "<H",
            rawdata,
            tsBytes,
            (self.basic_header["BytesInDataPackets"],),
        )

        # identify packet indices by type. if packet type is found, typecast rawdata into meaningful data arrays
        # neural and analog input data:
        neuralPackets = [
            idx
            for idx, element in enumerate(PacketID)
            if NEURAL_PACKET_ID_MIN <= element <= NEURAL_PACKET_ID_MAX
        ]
        if len(neuralPackets) > 0:
            ChannelID = PacketID
            if type(elec_ids) is list:
                elecindices = [
                    idx
                    for idx, element in enumerate(ChannelID[neuralPackets])
                    if element in elec_ids
                ]
                neuralPackets = [neuralPackets[index] for index in elecindices]

            spikeUnit = np.ndarray(
                (nPackets,),
                "<B",
                rawdata,
                tsBytes + 2,
                (self.basic_header["BytesInDataPackets"],),
            )
            output["spike_events"] = {
                "TimeStamps": list(ts[neuralPackets]),
                "Unit": list(spikeUnit[neuralPackets]),
                "Channel": list(ChannelID[neuralPackets]),
            }

            if wave_read == "read":
                wfs = np.ndarray(
                    (
                        nPackets,
                        int((self.basic_header["BytesInDataPackets"] - (tsBytes + 4))),
                    ),
                    "<h",
                    rawdata,
                    tsBytes + 4,
                    (self.basic_header["BytesInDataPackets"], 0),
                )
                output["spike_events"].update({"Waveforms": wfs[neuralPackets, :]})

        # digital events, i.e. digital inputs and serial inputs
        digiPackets = [
            idx for idx, element in enumerate(PacketID) if element == DIGITAL_PACKET_ID
        ]
        if len(digiPackets) > 0:
            insertionReason = np.ndarray(
                (nPackets,),
                "<B",
                rawdata,
                tsBytes + 2,
                (self.basic_header["BytesInDataPackets"],),
            )
            digiVals = np.ndarray(
                (nPackets,),
                "<I",
                rawdata,
                tsBytes + 4,
                (self.basic_header["BytesInDataPackets"],),
            )
            output["digital_events"] = {
                "TimeStamps": list(ts[digiPackets]),
                "InsertionReason": list(insertionReason[digiPackets]),
                "UnparsedData": list(digiVals[digiPackets]),
            }

        # comments + NeuroMotive events that are stored like comments
        commentPackets = [
            idx for idx, element in enumerate(PacketID) if element == COMMENT_PACKET_ID
        ]
        if len(commentPackets) > 0:
            charSet = np.ndarray(
                (nPackets,),
                "<B",
                rawdata,
                tsBytes + 2,
                (self.basic_header["BytesInDataPackets"],),
            )
            tsStarted = np.ndarray(
                (nPackets,),
                "<I",
                rawdata,
                tsBytes + 4,
                (self.basic_header["BytesInDataPackets"],),
            )
            charSet = charSet[commentPackets]
            charSetList = np.array([None] * len(charSet))
            ANSIPackets = [
                idx for idx, element in enumerate(charSet) if element == CHARSET_ANSI
            ]
            if len(ANSIPackets) > 0:
                charSetList[ANSIPackets] = "ANSI"
            UTFPackets = [
                idx for idx, element in enumerate(charSet) if element == CHARSET_UTF
            ]
            if len(UTFPackets) > 0:
                charSetList[UTFPackets] = "UTF "

            # need to transfer comments from neuromotive. identify region of interest (ROI) events...
            ROIPackets = [
                idx for idx, element in enumerate(charSet) if element == CHARSET_ROI
            ]

            lcomment = self.basic_header["BytesInDataPackets"] - tsBytes - 10
            comments = np.chararray(
                (nPackets, lcomment),
                1,
                False,
                rawdata,
                tsBytes + 8,
                (self.basic_header["BytesInDataPackets"], 1),
            )

            # extract only the "true" comments, distinct from ROI packets
            trueComments = np.setdiff1d(
                list(range(0, len(commentPackets) - 1)), ROIPackets
            )
            trueCommentsidx = np.asarray(commentPackets)[trueComments]
            textComments = comments[trueCommentsidx]
            textComments[:, -1] = "$"
            stringarray = textComments.tostring()
            stringvector = stringarray.decode("latin-1")
            stringvector = stringvector[0:-1]
            validstrings = stringvector.replace("\x00", "")
            commentsFinal = validstrings.split("$")

            # Remove the ROI comments from the list
            subsetInds = list(
                set(list(range(0, len(charSetList) - 1))) - set(ROIPackets)
            )

            output["comments"] = {
                "TimeStamps": list(ts[trueCommentsidx]),
                "TimeStampsStarted": list(tsStarted[trueCommentsidx]),
                "Data": commentsFinal,
                "CharSet": list(charSetList[subsetInds]),
            }

            # parsing and outputing ROI events
            if len(ROIPackets) > 0:
                nmPackets = np.asarray(ROIPackets)
                nmCommentsidx = np.asarray(commentPackets)[ROIPackets]
                nmcomments = comments[nmCommentsidx]
                nmcomments[:, -1] = ":"
                nmstringarray = nmcomments.tostring()
                nmstringvector = nmstringarray.decode("latin-1")
                nmstringvector = nmstringvector[0:-1]
                nmvalidstrings = nmstringvector.replace("\x00", "")
                nmcommentsFinal = nmvalidstrings.split(":")
                ROIfields = [l.split(":") for l in ":".join(nmcommentsFinal).split(":")]
                ROIfieldsRS = np.reshape(ROIfields, (len(ROIPackets), 5))
                output["tracking_events"] = {
                    "TimeStamps": list(ts[nmCommentsidx]),
                    "ROIName": list(ROIfieldsRS[:, 0]),
                    "ROINumber": list(ROIfieldsRS[:, 1]),
                    "Event": list(ROIfieldsRS[:, 2]),
                    "Frame": list(ROIfieldsRS[:, 3]),
                }

        # NeuroMotive video syncronization packets
        vidsyncPackets = [
            idx
            for idx, element in enumerate(PacketID)
            if element == VIDEO_SYNC_PACKET_ID
        ]
        if len(vidsyncPackets) > 0:
            fileNumber = np.ndarray(
                (nPackets,),
                "<H",
                rawdata,
                tsBytes + 2,
                (self.basic_header["BytesInDataPackets"],),
            )
            frameNumber = np.ndarray(
                (nPackets,),
                "<I",
                rawdata,
                tsBytes + 4,
                (self.basic_header["BytesInDataPackets"],),
            )
            elapsedTime = np.ndarray(
                (nPackets,),
                "<I",
                rawdata,
                tsBytes + 8,
                (self.basic_header["BytesInDataPackets"],),
            )
            sourceID = np.ndarray(
                (nPackets,),
                "<I",
                rawdata,
                tsBytes + 12,
                (self.basic_header["BytesInDataPackets"],),
            )
            output["video_sync_events"] = {
                "TimeStamps": list(ts[vidsyncPackets]),
                "FileNumber": list(fileNumber[vidsyncPackets]),
                "FrameNumber": list(frameNumber[vidsyncPackets]),
                "ElapsedTime": list(elapsedTime[vidsyncPackets]),
                "SourceID": list(sourceID[vidsyncPackets]),
            }

        # Neuromotive object tracking packets
        trackingPackets = [
            idx for idx, element in enumerate(PacketID) if element == TRACKING_PACKET_ID
        ]
        if len(trackingPackets) > 0:
            trackerObjs = [
                sub["VideoSource"]
                for sub in self.extended_headers
                if sub["PacketID"] == "TRACKOBJ"
            ]
            trackerIDs = [
                sub["TrackableID"]
                for sub in self.extended_headers
                if sub["PacketID"] == "TRACKOBJ"
            ]
            output["tracking"] = {
                "TrackerIDs": trackerIDs,
                "TrackerTypes": [
                    sub["TrackableType"]
                    for sub in self.extended_headers
                    if sub["PacketID"] == "TRACKOBJ"
                ],
            }
            parentID = np.ndarray(
                (nPackets,),
                "<H",
                rawdata,
                tsBytes + 2,
                (self.basic_header["BytesInDataPackets"],),
            )
            nodeID = np.ndarray(
                (nPackets,),
                "<H",
                rawdata,
                tsBytes + 4,
                (self.basic_header["BytesInDataPackets"],),
            )
            nodeCount = np.ndarray(
                (nPackets,),
                "<H",
                rawdata,
                tsBytes + 6,
                (self.basic_header["BytesInDataPackets"],),
            )
            markerCount = np.ndarray(
                (nPackets,),
                "<H",
                rawdata,
                tsBytes + 8,
                (self.basic_header["BytesInDataPackets"],),
            )
            bodyPointsX = np.ndarray(
                (nPackets,),
                "<H",
                rawdata,
                tsBytes + 10,
                (self.basic_header["BytesInDataPackets"],),
            )
            bodyPointsY = np.ndarray(
                (nPackets,),
                "<H",
                rawdata,
                tsBytes + 12,
                (self.basic_header["BytesInDataPackets"],),
            )

            # Need to parse by the tracker object to create clean outputs
            R = 0
            E = 0
            for i in range(0, len(trackerObjs)):
                indices = [
                    idx
                    for idx, element in enumerate(nodeID[trackingPackets])
                    if element == i
                ]

                # Static objects create single rectangles that only get sent over into the file once
                if len(indices) == 1:
                    if trackerObjs[i] == "TrackingROI":
                        trackerObjs[i] = trackerObjs[i] + str(R)
                        R += 1
                    elif trackerObjs[i] == "EventROI":
                        trackerObjs[i] = trackerObjs[i] + str(E)
                        E += 1
                    bodyPointsX = np.ndarray(
                        (nPackets, 4),
                        "<H",
                        rawdata,
                        tsBytes + 10,
                        (self.basic_header["BytesInDataPackets"], 2),
                    )
                    bodyPointsY = np.ndarray(
                        (nPackets, 4),
                        "<H",
                        rawdata,
                        tsBytes + 12,
                        (self.basic_header["BytesInDataPackets"], 2),
                    )
                selectedIndices = [trackingPackets[index] for index in indices]
                tempDict = {
                    "TimeStamps": list(ts[selectedIndices]),
                    "ParentID": list(parentID[selectedIndices]),
                    "NodeCount": list(nodeCount[selectedIndices]),
                    "MarkerCount": list(markerCount[selectedIndices]),
                    "X": list(bodyPointsX[selectedIndices]),
                    "Y": list(bodyPointsY[selectedIndices]),
                }
                output["tracking"].update({trackerObjs[i]: tempDict})
            output["tracking"].update({"TrackerObjs": trackerObjs})

        # patient trigger events
        buttonPackets = [
            idx for idx, element in enumerate(PacketID) if element == BUTTON_PACKET_ID
        ]
        if len(buttonPackets) > 0:
            trigType = np.ndarray(
                (nPackets,),
                "<H",
                rawdata,
                tsBytes + 2,
                (self.basic_header["BytesInDataPackets"],),
            )
            output["PatientTrigger"] = {
                "TimeStamps": list(ts[buttonPackets]),
                "TriggerType": list(trigType[buttonPackets]),
            }

        # configuration packets
        configPackets = [
            idx
            for idx, element in enumerate(PacketID)
            if element == CONFIGURATION_PACKET_ID
        ]
        if len(configPackets) > 0:
            changeType = np.ndarray(
                (nPackets,),
                "<H",
                rawdata,
                tsBytes + 2,
                (self.basic_header["BytesInDataPackets"],),
            )
            output["reconfig"] = {
                "TimeStamps": list(ts[configPackets]),
                "ChangeType": list(ts[configPackets]),
            }

        return output

    def processroicomments(
        self, comments
    ):  # obsolete in v2.0.0+, ROI comments come out parsed from NevFile.getdata()
        """
        used to process the comment data packets associated with NeuroMotive region of interest enter/exit events.
        requires that read_data() has already been run.
        :return: roi_events:   a dictionary of regions, enter timestamps, and exit timestamps for each region
        """

        roi_events = {"Regions": [], "EnterTimeStamps": [], "ExitTimeStamps": []}

        for i in range(len(comments["TimeStamps"])):
            if comments["CharSet"][i] == "NeuroMotive ROI":

                temp_data = pack("<I", comments["Data"][i])
                roi = unpack_from("<B", temp_data)[0]
                event = unpack_from("<B", temp_data, 1)[0]

                # Determine the label of the region source
                source_label = next(
                    d["VideoSource"]
                    for d in self.extended_headers
                    if d["TrackableID"] == roi
                )

                # update the timestamps for events
                if source_label in roi_events["Regions"]:
                    idx = roi_events["Regions"].index(source_label)
                else:
                    idx = -1
                    roi_events["Regions"].append(source_label)
                    roi_events["EnterTimeStamps"].append([])
                    roi_events["ExitTimeStamps"].append([])

                if event == ENTER_EVENT:
                    roi_events["EnterTimeStamps"][idx].append(comments["TimeStamp"][i])
                elif event == EXIT_EVENT:
                    roi_events["ExitTimeStamps"][idx].append(comments["TimeStamp"][i])

        return roi_events

    def close(self):
        name = self.datafile.name
        self.datafile.close()
        print("\n" + name.split("/")[-1] + " closed")
        
    # add "enter" and "exit" methods for compatibility with "with":
    def __enter__(self):
        return self # thankfully this returns a reference, not a copy
    
    def __exit__(self,*args):
        self.close()


class NsxFile:
    """
    attributes and methods for all BR continuous data files.  Initialization opens the file and extracts the
    basic header information.
    """

    def __init__(self, datafile=""):

        self.datafile = datafile
        self.basic_header = {}
        self.extended_headers = []
        
        self.openfile()
        
    def openfile(self):
        # Run openfilecheck and open the file passed or allow user to browse to one
        self.datafile = openfilecheck(
            "rb",
            file_name=self.datafile,
            file_ext=".ns*",
            file_type="Blackrock NSx Files",
        )

        # Determine File ID to determine if File Spec 2.1
        self.basic_header["FileTypeID"] = bytes.decode(self.datafile.read(8), "latin-1")

        # Extract basic and extended header information based on File Spec
        if self.basic_header["FileTypeID"] == "NEURALSG":
            self.basic_header.update(
                processheaders(self.datafile, nsx_header_dict["basic_21"])
            )
            self.basic_header["FileSpec"] = "2.1"
            self.basic_header["TimeStampResolution"] = 30000
            self.basic_header["BytesInHeader"] = (
                32 + 4 * self.basic_header["ChannelCount"]
            )
            shape = (1, self.basic_header["ChannelCount"])
            self.basic_header["ChannelID"] = list(
                np.fromfile(
                    file=self.datafile,
                    dtype=np.uint32,
                    count=self.basic_header["ChannelCount"],
                ).reshape(shape)[0]
            )
        else:
            self.basic_header.update(
                processheaders(self.datafile, nsx_header_dict["basic"])
            )
            for i in range(self.basic_header["ChannelCount"]):
                self.extended_headers.append(
                    processheaders(self.datafile, nsx_header_dict["extended"])
                )

    def getdata(
        self,
        elec_ids="all",
        start_time_s=0,
        data_time_s="all",
        downsample=1,
        zeropad=False,
    ):
        """
        This function is used to return a set of data from the NSx datafile.

        :param elec_ids:      [optional] {list}  List of elec_ids to extract (e.g., [13])
        :param start_time_s:  [optional] {float} Starting time for data extraction (e.g., 1.0)
        :param data_time_s:   [optional] {float} Length of time of data to return (e.g., 30.0)
        :param downsample:    [optional] {int}   Downsampling factor (e.g., 2)
        :return: output:      {Dictionary} of:  data_headers: {list}        dictionaries of all data headers
                                                elec_ids:     {list}        elec_ids that were extracted (sorted)
                                                start_time_s: {float}       starting time for data extraction
                                                data_time_s:  {float}       length of time of data returned
                                                downsample:   {int}         data downsampling factor
                                                samp_per_s:   {float}       output data samples per second
                                                data:         {numpy array} continuous data in a 2D numpy array

        Parameters: elec_ids, start_time_s, data_time_s, and downsample are not mandatory.  Defaults will assume all
        electrodes and all data points starting at time(0) are to be read. Data is returned as a numpy 2d array
        with each row being the data set for each electrode (e.g. output['data'][0] for output['elec_ids'][0]).
        """

        # Safety checks
        start_time_s = check_starttime(start_time_s)
        data_time_s = check_datatime(data_time_s)
        downsample = check_downsample(downsample)
        elec_ids = check_elecid(elec_ids)

        # initialize parameters
        output = dict()
        output["elec_ids"] = elec_ids
        output["start_time_s"] = float(start_time_s)
        output["data_time_s"] = data_time_s
        output["downsample"] = downsample
        output["data"] = []
        output["data_headers"] = []
        output["ExtendedHeaderIndices"] = []

        datafile_samp_per_sec = (
            self.basic_header["TimeStampResolution"] / self.basic_header["Period"]
        )
        data_pt_size = self.basic_header["ChannelCount"] * DATA_BYTE_SIZE
        elec_id_indices = []
        front_end_idxs = []
        analog_input_idxs = []
        front_end_idx_cont = True
        analog_input_idx_cont = True
        hit_start = False
        hit_stop = False
        d_ptr = {"BoH": [], "BoD": []}
        # Move file position to start of datafile (if read before, may not be here anymore)
        self.datafile.seek(self.basic_header["BytesInHeader"], 0)

        # Based on FileSpec set other parameters
        if self.basic_header["FileSpec"] == "2.1":
            output["elec_ids"] = self.basic_header["ChannelID"]
            output["data_headers"].append({})
            output["data_headers"][0]["Timestamp"] = TIMESTAMP_NULL_21
            output["data_headers"][0]["NumDataPoints"] = (
                ospath.getsize(self.datafile.name) - self.datafile.tell()
            ) // (DATA_BYTE_SIZE * self.basic_header["ChannelCount"])
        else:
            output["elec_ids"] = [d["ElectrodeID"] for d in self.extended_headers]

        # Determine start and stop index for data
        if start_time_s == START_TIME_DEF:
            start_idx = START_OFFSET_MIN
        else:
            start_idx = int(round(start_time_s * datafile_samp_per_sec))
        if data_time_s == DATA_TIME_DEF:
            stop_idx = STOP_OFFSET_MIN
        else:
            stop_idx = int(round((start_time_s + data_time_s) * datafile_samp_per_sec))

        # If a subset of electrodes is requested, error check, determine elec indices, and reduce headers
        if elec_ids != ELEC_ID_DEF:
            elec_ids = check_dataelecid(elec_ids, output["elec_ids"])
            if not elec_ids:
                return output
            else:
                elec_id_indices = [output["elec_ids"].index(e) for e in elec_ids]
                output["elec_ids"] = elec_ids
        num_elecs = len(output["elec_ids"])

        # Determine extended header indices and idx for Front End vs. Analog Input channels
        if self.basic_header["FileSpec"] != "2.1":
            for i in range(num_elecs):
                idx = next(
                    item
                    for (item, d) in enumerate(self.extended_headers)
                    if d["ElectrodeID"] == output["elec_ids"][i]
                )
                output["ExtendedHeaderIndices"].append(idx)

                if self.extended_headers[idx]["PhysicalConnector"] < 5:
                    front_end_idxs.append(i)
                else:
                    analog_input_idxs.append(i)

            # Determine if front_end_idxs and analog_idxs are contiguous (default = False)
            if any(np.diff(np.array(front_end_idxs)) != 1):
                front_end_idx_cont = False
            if any(np.diff(np.array(analog_input_idxs)) != 1):
                analog_input_idx_cont = False

        # Pre-allocate output data based on data packet info (timestamp + num pts) and/or data_time_s
        # 1) Determine number of samples in all data packets to set possible number of output pts
        # 1a) For file spec > 2.1, get to last data packet quickly to determine total possible output length
        # 2) If possible output length is bigger than requested, set output based on requested
        if self.basic_header["FileSpec"] == "2.1":
            timestamp = TIMESTAMP_NULL_21
            num_data_pts = output["data_headers"][0]["NumDataPoints"]
        else:
            while self.datafile.tell() < ospath.getsize(self.datafile.name):
                d_ptr["BoH"].append(self.datafile.tell())
                self.datafile.seek(1, 1)
                if self.basic_header["FileSpec"] == "3.0":
                    timestamp = unpack("<Q", self.datafile.read(8))[0]
                else:
                    timestamp = unpack("<I", self.datafile.read(4))[0]
                num_data_pts = unpack("<I", self.datafile.read(4))[0]
                d_ptr["BoD"].append(self.datafile.tell())
                output["data_headers"].append(
                    {"Timestamp": timestamp, "NumDataPoints": num_data_pts}
                )
                self.datafile.seek(
                    num_data_pts * self.basic_header["ChannelCount"] * DATA_BYTE_SIZE, 1
                )

        # stop_idx_output = ceil(timestamp / self.basic_header['Period']) + num_data_pts
        # if data_time_s != DATA_TIME_DEF and stop_idx < stop_idx_output:  stop_idx_output = stop_idx
        # total_samps = int(ceil((stop_idx_output - start_idx) / downsample))
        for x in range(len(output["data_headers"])):
            if (
                output["data_headers"][x]["NumDataPoints"]
                * self.basic_header["ChannelCount"]
                * DATA_BYTE_SIZE
            ) > DATA_PAGING_SIZE:
                print(
                    "\nOutput data requested is larger than 1 GB, attempting to preallocate output now"
                )

            # If data output is bigger than available, let user know this is too big and they must request at least one of:
            # subset of electrodes, subset of data, or use savensxsubset to smaller file sizes, otherwise, pre-allocate data
            self.datafile.seek(d_ptr["BoD"][x])
            data_length = output["data_headers"][x]["NumDataPoints"]
            recorded_data_bytes = data_length * num_elecs * DATA_BYTE_SIZE
            recorded_data = self.datafile.read(recorded_data_bytes)
            if zeropad is True:
                padsize = ceil(
                    output["data_headers"][x]["Timestamp"] / self.basic_header["Period"]
                )
                data_length += padsize
                zero_array = bytes(np.zeros((padsize * num_elecs, 1), dtype=np.short))
            try:
                np.zeros((data_length, num_elecs), dtype=np.short)
            except MemoryError as err:
                err.args += (
                    " Output data size requested is larger than available memory. Use the parameters\n"
                    "              for getdata(), e.g., 'elec_ids', to request a subset of the data or use\n"
                    "              NsxFile.savesubsetnsx() to create subsets of the main nsx file\n",
                )
                raise
            if zeropad is True:
                recorded_data = zero_array + recorded_data
            output["data"].append(
                np.ndarray((data_length, num_elecs), "<h", recorded_data)
            )

            # Reset file position to start of data header #1, loop through all data packets, process header, and add data
        #     self.datafile.seek(self.basic_header['BytesInHeader'], 0)
        #     while not hit_stop:
        #
        #     # Read header, check to make sure the header is valid (ie Header field != 0).  There is currently a
        #     # bug with the NSP where pausing creates a 0 sample packet before the next real data packet, these need to
        #     # be skipped, including any tiny packets that have less samples than downsample
        #     if self.basic_header['FileSpec'] != '2.1':
        #         output['data_headers'].append(processheaders(self.datafile, nsx_header_dict['data']))
        #         if output['data_headers'][-1]['Header'] == 0:  print('Invalid Header.  File may be corrupt')
        #         if output['data_headers'][-1]['NumDataPoints'] < downsample:
        #             self.datafile.seek(self.basic_header['ChannelCount'] * output['data_headers'][-1]['NumDataPoints']
        #                                * DATA_BYTE_SIZE, 1)
        #             continue
        #
        #     # Determine sample value for current packet timestamp
        #     # timestamp_sample = int(round(output['data_headers'][-1]['Timestamp'] / self.basic_header['Period']))
        #
        #     # For now, we need a patch for file sync which syncs 2 NSP clocks, starting a new data packet which
        #     # may be backwards in time wrt the end of data packet 1.  Thus, when this happens, we need to treat
        #     # data packet 2 as if it was 1, and start this process over.
        #     if timestamp_sample < d_ptr:
        #         d_ptr = 0
        #         hit_start = False
        #         output['data_headers'] = []
        #         self.datafile.seek(-9, 1)
        #         continue
        #
        #     # Check to see if stop index is before the first data packet
        #     if len(output['data_headers']) == 1 and (STOP_OFFSET_MIN < stop_idx < timestamp_sample):
        #         print("\nData requested is before any data was saved, which starts at t = {0:.6f} s".format(
        #             output['data_headers'][0]['Timestamp'] / self.basic_header['TimeStampResolution']))
        #         return
        #
        #     # For the first data packet to be read
        #     if not hit_start:
        #
        #         # Check for starting point of data request
        #         start_offset = start_idx - timestamp_sample
        #
        #         # If start_offset is outside of this packet, skip the current packet
        #         # if we've reached the end of file, break, otherwise continue to next packet
        #         if start_offset > output['data_headers'][-1]['NumDataPoints']:
        #             self.datafile.seek(output['data_headers'][-1]['NumDataPoints'] * data_pt_size, 1)
        #             if self.datafile.tell() == ospath.getsize(self.datafile.name): break
        #             else: continue
        #
        #         else:
        #             # If the start_offset is before the current packet, check to ensure that stop_index
        #             # is not also in the paused area, then create padded data for during pause time
        #             if start_offset < 0:
        #                 if STOP_OFFSET_MIN < stop_idx < timestamp_sample:
        #                     print("\nBecause of pausing, data section requested is during pause period")
        #                     return
        #                 else:
        #                     print("\nFirst data packet requested begins at t = {0:.6f} s, "
        #                           "initial section padded with zeros".format(
        #                            output['data_headers'][-1]['Timestamp'] / self.basic_header['TimeStampResolution']))
        #                     start_offset = START_OFFSET_MIN
        #                     d_ptr        = (timestamp_sample - start_idx) // downsample
        #             hit_start = True
        #
        #     # for all other packets
        #     else:
        #         # check to see if padded data is needed, including hitting the stop index
        #         if STOP_OFFSET_MIN < stop_idx < timestamp_sample:
        #             print("\nSection padded with zeros due to file pausing")
        #             hit_stop = True;  break
        #
        #         elif (timestamp_sample - start_idx) > d_ptr:
        #             print("\nSection padded with zeros due to file pausing")
        #             start_offset = START_OFFSET_MIN
        #             d_ptr        = (timestamp_sample - start_idx) // downsample
        #
        #     # Set number of samples to be read based on if start/stop sample is during data packet
        #     if STOP_OFFSET_MIN < stop_idx <= (timestamp_sample + output['data_headers'][-1]['NumDataPoints']):
        #         total_pts = stop_idx - timestamp_sample - start_offset
        #         hit_stop = True
        #     else:
        #         total_pts = output['data_headers'][-1]['NumDataPoints'] - start_offset
        #
        #     # Need current file position because memory map will reset file position
        #     curr_file_pos = self.datafile.tell()
        #
        #     # Determine starting position to read from memory map
        #     file_offset = int(curr_file_pos + start_offset * data_pt_size)
        #
        #     # Extract data no more than 1 GB at a time (or based on DATA_PAGING_SIZE)
        #     # Determine shape of data to map based on file sizing and position, then map it
        #     downsample_data_size = data_pt_size * downsample
        #     max_length           = (DATA_PAGING_SIZE // downsample_data_size) * downsample_data_size
        #     num_loops            = int(ceil(total_pts * data_pt_size / max_length))
        #
        #     for loop in range(num_loops):
        #         if loop == 0:
        #             if num_loops == 1:  num_pts = total_pts
        #             else:               num_pts = max_length // data_pt_size
        #
        #         else:
        #             file_offset += max_length
        #             if loop == (num_loops - 1):  num_pts = ((total_pts * data_pt_size) % max_length) // data_pt_size
        #             else:                        num_pts = max_length // data_pt_size
        #
        #         if num_loops != 1:  print('Data extraction requires paging: {0} of {1}'.format(loop + 1, num_loops))
        #
        #         num_pts = int(num_pts)
        #         shape   = (num_pts, self.basic_header['ChannelCount'])
        #         mm      = np.memmap(self.datafile, dtype=np.int16, mode='r', offset=file_offset, shape=shape)
        #
        #         # append data based on downsample slice and elec_ids indexing, then clear memory map
        #         if downsample != 1: mm = mm[::downsample]
        #         if elec_id_indices:
        #             output['data'][d_ptr:d_ptr + mm.shape[0]] = np.array(mm[:, elec_id_indices]).astype(np.float32)
        #         else:
        #             output['data'][d_ptr:d_ptr + mm.shape[0]] = np.array(mm).astype(np.float32)
        #         d_ptr += mm.shape[0]
        #         del mm
        #
        #     # Reset current file position for file position checking and possibly next header
        #     curr_file_pos += self.basic_header['ChannelCount'] * output['data_headers'][-1]['NumDataPoints'] \
        #                      * DATA_BYTE_SIZE
        #     self.datafile.seek(curr_file_pos, 0)
        #     if curr_file_pos == ospath.getsize(self.datafile.name):  hit_stop = True
        #
        # # Safety checks for start and stop times
        # if not hit_stop and start_idx > START_OFFSET_MIN:
        #     raise Exception('Error: End of file found before start_time_s')
        # elif not hit_stop and stop_idx:
        #     print("\n*** WARNING: End of file found before stop_time_s, returning all data in file")
        #
        # # Transpose the data so that it has entries based on each electrode, not each sample time
        # output['data'] = output['data'].transpose()
        #
        # # All data must be scaled based on scaling factors from extended header
        # if self.basic_header['FileSpec'] == '2.1':  output['data'] *= UV_PER_BIT_21
        # else:
        #     if front_end_idxs:
        #         if front_end_idx_cont:
        #             output['data'][front_end_idxs[0]:front_end_idxs[-1] + 1] *= \
        #                 getdigfactor(self.extended_headers, output['ExtendedHeaderIndices'][front_end_idxs[0]])
        #         else:
        #             for i in front_end_idxs:
        #                 output['data'][i] *= getdigfactor(self.extended_headers, output['ExtendedHeaderIndices'][i])
        #
        #     if analog_input_idxs:
        #         if analog_input_idx_cont:
        #             output['data'][analog_input_idxs[0]:analog_input_idxs[-1] + 1] *= \
        #                 getdigfactor(self.extended_headers, output['ExtendedHeaderIndices'][analog_input_idxs[0]])
        #         else:
        #             for i in analog_input_idxs:
        #                 output['data'][i] *= getdigfactor(self.extended_headers, output['ExtendedHeaderIndices'][i])
        #
        # # Update parameters based on data extracted
        # output['samp_per_s']  = float(datafile_samp_per_sec / downsample)
        # output['data_time_s'] = len(output['data'][0]) / output['samp_per_s']

        return output

    def savesubsetnsx(
        self, elec_ids="all", file_size=None, file_time_s=None, file_suffix=""
    ):
        """
        This function is used to save a subset of data based on electrode IDs, file sizing, or file data time.  If
        both file_time_s and file_size are passed, it will default to file_time_s and determine sizing accordingly.

        :param elec_ids:    [optional] {list}  List of elec_ids to extract (e.g., [13])
        :param file_size:   [optional] {int}   Byte size of each subset file to save (e.g., 1024**3 = 1 Gb). If nothing
                                                   is passed, file_size will be all data points.
        :param file_time_s: [optional] {float} Time length of data for each subset file, in seconds (e.g. 60.0).  If
                                                   nothing is passed, file_size will be used as default.
        :param file_suffix: [optional] {str}   Suffix to append to NSx datafile name for subset files.  If nothing is
                                                   passed, default will be "_subset".
        :return: None - None of the electrodes requested exist in the data
                 SUCCESS - All file subsets extracted and saved
        """

        # Initializations
        elec_id_indices = []
        file_num = 1
        pausing = False
        datafile_datapt_size = self.basic_header["ChannelCount"] * DATA_BYTE_SIZE
        self.datafile.seek(0, 0)

        # Run electrode id checks and set num_elecs
        elec_ids = check_elecid(elec_ids)
        if self.basic_header["FileSpec"] == "2.1":
            all_elec_ids = self.basic_header["ChannelID"]
        else:
            all_elec_ids = [x["ElectrodeID"] for x in self.extended_headers]

        if elec_ids == ELEC_ID_DEF:
            elec_ids = all_elec_ids
        else:
            elec_ids = check_dataelecid(elec_ids, all_elec_ids)
            if not elec_ids:
                return None
            else:
                elec_id_indices = [all_elec_ids.index(x) for x in elec_ids]

        num_elecs = len(elec_ids)

        # If file_size or file_time_s passed, check it and set file_sizing accordingly
        if file_time_s:
            if file_time_s and file_size:
                print(
                    "\nWARNING: Only one of file_size or file_time_s can be passed, defaulting to file_time_s."
                )
            file_size = int(
                num_elecs
                * DATA_BYTE_SIZE
                * file_time_s
                * self.basic_header["TimeStampResolution"]
                / self.basic_header["Period"]
            )
            if self.basic_header["FileSpec"] == "2.1":
                file_size += 32 + 4 * num_elecs
            else:
                file_size += (
                    NSX_BASIC_HEADER_BYTES_22 + NSX_EXT_HEADER_BYTES_22 * num_elecs + 5
                )
            print(
                "\nBased on timing request, file size will be {0:d} Mb".format(
                    int(file_size / 1024**2)
                )
            )
        elif file_size:
            file_size = check_filesize(file_size)

        # Create and open subset file as writable binary, if it already exists ask user for overwrite permission
        file_name, file_ext = ospath.splitext(self.datafile.name)
        if file_suffix:
            file_name += "_" + file_suffix
        else:
            file_name += "_subset"

        if ospath.isfile(file_name + "_000" + file_ext):
            if "y" != input(
                "\nFile '"
                + file_name.split("/")[-1]
                + "_xxx"
                + file_ext
                + "' already exists, overwrite [y/n]: "
            ):
                print("\nExiting, no overwrite, returning None")
                return None
            else:
                print("\n*** Overwriting existing subset files ***")

        subset_file = open(file_name + "_000" + file_ext, "wb")
        print("\nWriting subset file: " + ospath.split(subset_file.name)[1])

        # For file spec 2.1:
        #   1) copy the first 28 bytes from the datafile (these are unchanged)
        #   2) write subset channel count and channel ID to file
        #   3) skip ahead in datafile the number of bytes in datafile ChannelCount(4) plus ChannelID (4*ChannelCount)
        if self.basic_header["FileSpec"] == "2.1":
            subset_file.write(self.datafile.read(28))
            subset_file.write(np.array(num_elecs).astype(np.uint32).tobytes())
            subset_file.write(np.array(elec_ids).astype(np.uint32).tobytes())
            self.datafile.seek(4 + 4 * self.basic_header["ChannelCount"], 1)

        # For file spec 2.2 and above
        #    1) copy the first 10 bytes from the datafile (unchanged)
        #    2) write subset bytes-in-headers and skip 4 bytes in datafile, noting position of this for update later
        #    3) copy the next 296 bytes from datafile (unchanged)
        #    4) write subset channel-count value and skip 4 bytes in datafile
        #    5) append extended headers based on the channel ID.  Must read the first 4 bytes, determine if correct
        #          Channel ID, repack first 4 bytes, write to disk, then copy remaining 62 (66-4) bytes
        else:
            subset_file.write(self.datafile.read(10))
            bytes_in_headers = (
                NSX_BASIC_HEADER_BYTES_22 + NSX_EXT_HEADER_BYTES_22 * num_elecs
            )
            num_pts_header_pos = bytes_in_headers + 5
            subset_file.write(np.array(bytes_in_headers).astype(np.uint32).tobytes())
            self.datafile.seek(4, 1)
            subset_file.write(self.datafile.read(296))
            subset_file.write(np.array(num_elecs).astype(np.uint32).tobytes())
            self.datafile.seek(4, 1)

            for i in range(len(self.extended_headers)):
                h_type = self.datafile.read(2)
                chan_id = self.datafile.read(2)
                if unpack("<H", chan_id)[0] in elec_ids:
                    subset_file.write(h_type)
                    subset_file.write(chan_id)
                    subset_file.write(self.datafile.read(62))
                else:
                    self.datafile.seek(62, 1)

        # For all file types, loop through all data packets, extracting data based on page sizing
        while self.datafile.tell() != ospath.getsize(self.datafile.name):

            # pull and set data packet header info
            if self.basic_header["FileSpec"] == "2.1":
                packet_pts = (
                    ospath.getsize(self.datafile.name) - self.datafile.tell()
                ) / (DATA_BYTE_SIZE * self.basic_header["ChannelCount"])
            else:
                header_binary = self.datafile.read(1)
                timestamp_binary = self.datafile.read(4)
                packet_pts_binary = self.datafile.read(4)
                packet_pts = unpack("<I", packet_pts_binary)[0]
                if packet_pts == 0:
                    continue

                subset_file.write(header_binary)
                subset_file.write(timestamp_binary)
                subset_file.write(packet_pts_binary)

            # get current file position and set loop parameters
            datafile_pos = self.datafile.tell()
            file_offset = datafile_pos
            mm_length = (
                DATA_PAGING_SIZE // datafile_datapt_size
            ) * datafile_datapt_size
            num_loops = int(ceil(packet_pts * datafile_datapt_size / mm_length))
            packet_read_pts = 0
            subset_file_pkt_pts = 0

            # Determine shape of data to map based on file sizing and position, map it, then append to file
            for loop in range(num_loops):
                if loop == 0:
                    if num_loops == 1:
                        num_pts = packet_pts
                    else:
                        num_pts = mm_length // datafile_datapt_size

                else:
                    file_offset += mm_length
                    if loop == (num_loops - 1):
                        num_pts = (
                            (packet_pts * datafile_datapt_size) % mm_length
                        ) // datafile_datapt_size
                    else:
                        num_pts = mm_length // datafile_datapt_size

                shape = (int(num_pts), self.basic_header["ChannelCount"])
                mm = np.memmap(
                    self.datafile,
                    dtype=np.int16,
                    mode="r",
                    offset=file_offset,
                    shape=shape,
                )
                if elec_id_indices:
                    mm = mm[:, elec_id_indices]
                start_idx = 0

                # Determine if we need to start an additional file
                if file_size and (file_size - subset_file.tell()) < DATA_PAGING_SIZE:

                    # number of points we can possibly write to current subset file
                    pts_can_add = (
                        int(
                            (file_size - subset_file.tell())
                            // (num_elecs * DATA_BYTE_SIZE)
                        )
                        + 1
                    )
                    stop_idx = start_idx + pts_can_add

                    # If the pts remaining are less than exist in the data, we'll need an additional subset file
                    while pts_can_add < num_pts:

                        # Write pts to disk, set old file name, update pts in packet, and close last subset file
                        if elec_id_indices:
                            subset_file.write(
                                np.array(mm[start_idx:stop_idx]).tobytes()
                            )
                        else:
                            subset_file.write(mm[start_idx:stop_idx])
                        prior_file_name = subset_file.name
                        prior_file_pkt_pts = subset_file_pkt_pts + pts_can_add
                        subset_file.close()

                        # We need to copy header information from last subset file and adjust some headers.
                        # For file spec 2.1, this is just the basic header.
                        # For file spec 2.2 and above:
                        #    1) copy basic and extended headers
                        #    2) create data packet header with new timestamp and num data points (dummy numpts value)
                        #    3) overwrite the number of data points in the old file last header packet with true value
                        prior_file = open(prior_file_name, "rb+")
                        if file_num < 10:
                            numstr = "_00" + str(file_num)
                        elif 10 <= file_num < 100:
                            numstr = "_0" + str(file_num)
                        else:
                            numstr = "_" + str(file_num)
                        subset_file = open(file_name + numstr + file_ext, "wb")
                        print(
                            "Writing subset file: " + ospath.split(subset_file.name)[1]
                        )

                        if self.basic_header["FileSpec"] == "2.1":
                            subset_file.write(prior_file.read(32 + 4 * num_elecs))
                        else:
                            subset_file.write(prior_file.read(bytes_in_headers))
                            subset_file.write(header_binary)
                            timestamp_new = (
                                unpack("<I", timestamp_binary)[0]
                                + (packet_read_pts + pts_can_add)
                                * self.basic_header["Period"]
                            )
                            subset_file.write(
                                np.array(timestamp_new).astype(np.uint32).tobytes()
                            )
                            subset_file.write(
                                np.array(num_pts - pts_can_add)
                                .astype(np.uint32)
                                .tobytes()
                            )

                            prior_file.seek(num_pts_header_pos, 0)
                            prior_file.write(
                                np.array(prior_file_pkt_pts).astype(np.uint32).tobytes()
                            )

                            num_pts_header_pos = bytes_in_headers + 5

                        # Close old file and update parameters
                        prior_file.close()
                        packet_read_pts += pts_can_add
                        start_idx += pts_can_add
                        num_pts -= pts_can_add
                        file_num += 1
                        subset_file_pkt_pts = 0
                        pausing = False

                        pts_can_add = (
                            int(
                                (file_size - subset_file.tell())
                                // (num_elecs * DATA_BYTE_SIZE)
                            )
                            + 1
                        )
                        stop_idx = start_idx + pts_can_add

                # If no additional file needed, write remaining data to disk, update parameters, and clear memory map
                if elec_id_indices:
                    subset_file.write(np.array(mm[start_idx:]).tobytes())
                else:
                    subset_file.write(mm[start_idx:])
                packet_read_pts += num_pts
                subset_file_pkt_pts += num_pts
                del mm

            # Update num_pts header position for each packet, while saving last packet num_pts_header_pos for later
            if self.basic_header["FileSpec"] != "2.1":
                curr_hdr_num_pts_pos = num_pts_header_pos
                num_pts_header_pos += (
                    4 + subset_file_pkt_pts * num_elecs * DATA_BYTE_SIZE + 5
                )

            # Because memory map resets the file position, reset position in datafile
            datafile_pos += (
                self.basic_header["ChannelCount"] * packet_pts * DATA_BYTE_SIZE
            )
            self.datafile.seek(datafile_pos, 0)

            # If using file_timing and there is pausing in data (multiple packets), let user know
            if (
                file_time_s
                and not pausing
                and (self.datafile.tell() != ospath.getsize(self.datafile.name))
            ):
                pausing = True
                print(
                    "\n*** Because of pausing in original datafile, this file may be slightly time shorter\n"
                    "       than others, and will contain multiple data packets offset in time\n"
                )

            # Update last data header packet num data points accordingly (spec != 2.1)
            if self.basic_header["FileSpec"] != "2.1":
                subset_file_pos = subset_file.tell()
                subset_file.seek(curr_hdr_num_pts_pos, 0)
                subset_file.write(
                    np.array(subset_file_pkt_pts).astype(np.uint32).tobytes()
                )
                subset_file.seek(subset_file_pos, 0)

        # Close subset file and return success
        subset_file.close()
        print("\n *** All subset files written to disk and closed ***")
        return "SUCCESS"

    def close(self):
        name = self.datafile.name
        self.datafile.close()
        print("\n" + name.split("/")[-1] + " closed")
        
    # add "enter" and "exit" methods for compatibility with "with":
    def __enter__(self):
        return self # thankfully this returns a reference, not a copy 
    
    def __exit__(self,*args):
        self.close()
