# -*- coding: utf-8 -*-
"""
Example of how to extract data from a Blackrock Nsx data file and save new subset Nsx data files
current version: 1.1.1 --- 07/22/2016

@author: Mitch Frankel - Blackrock Microsystems
"""

"""
Version History:
v1.0.0 - 07/08/2016 - initial release - requires brpylib v1.1.0 or higher
v1.1.0 - 07/12/2016 - addition of version checking for brpylib starting with v1.2.0
v1.1.1 - 07/22/2016 - minor modifications to use close() functionality of NsxFile class
"""

# Imports
from brpylib import NsxFile, brpylib_ver

# Version control
brpylib_ver_req = "1.2.1"
if brpylib_ver.split('.') < brpylib_ver_req.split('.'):
    raise Exception("requires brpylib " + brpylib_ver_req + " or higher, please use latest version")

# Inits
datafile = 'D:/Dropbox/BlackrockDB/software/sampledata/The Most Perfect Data in the WWWorld/' \
            'sampleData.ns6'

# Open file and extract headers
brns_file = NsxFile(datafile)

# save a subset of data based on elec_ids
brns_file.savesubsetnsx(elec_ids=[1, 2, 5, 15, 20, 200], file_suffix='elec_subset')

# save a subset of data based on file sizing (100 Mb)
brns_file.savesubsetnsx(file_size=(1024**2) * 100, file_suffix='size_subset')

# save a subset of data based on file timing
brns_file.savesubsetnsx(file_time_s=30, file_suffix='time_subset')

# save a subset of data based on elec_ids and timing
brns_file.savesubsetnsx(elec_ids=[1, 2, 5, 15, 20, 200], file_time_s=30, file_suffix='elecAndTime_subset')

# Close the original datafile
brns_file.close()
