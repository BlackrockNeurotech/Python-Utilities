"""
Random functions that may be useful elsewhere (or necessary)
current version: 1.3.0 --- 05/08/2024

@author: Mitch Frankel - Blackrock Microsystems

Version History:
v1.0.0 - 07/05/2016 - initial release
v1.1.0 - 07/12/2016 - minor editing changes to print statements and addition of version control
v1.2.0 - 08/04/2016 - minor modifications to allow use of Python 2.6+
v1.3.0 - 05/08/2024 - removed Qt dependency, replaced with tkinter
"""
from os import getcwd, path
from tkinter import Tk
from tkinter.filedialog import askopenfile

# Version control
brmiscfxns_ver = "1.3.0"

# Patch for use with Python 2.6+
try:
    input = raw_input
except NameError:
    pass


def openfilecheck(open_mode, file_name="", file_ext="", file_type=""):
    """
    :param open_mode: {str} method to open the file (e.g., 'rb' for binary read only)
    :param file_name: [optional] {str} full path of file to open
    :param file_ext:  [optional] {str} file extension (e.g., '.nev')
    :param file_type: [optional] {str} file type for use when browsing for file (e.g., 'Blackrock NEV Files')
    :return: {file} opened file
    """

    while True:
        if not file_name:  # no file name passed
            if not file_type: # no extension passed
                file_type = ("*.ns1", "*.ns2", "*.ns3", "*.ns4", "*.ns5", "*.ns6")
            # Ask user to specify a file path or browse
            Tk().withdraw()
            file_name = askopenfile(
                title="Select original file",
                initialdir=getcwd(),
                filetypes=[("NSx Files", file_type)],
                )
            file_name = file_name.name

        # Ensure file exists (really needed for users type entering)
        if path.isfile(file_name):
            # Ensure given file matches file_ext
            if file_ext:
                _, fext = path.splitext(file_name)

                # check for * in extension
                if file_ext[-1] == "*":
                    test_extension = file_ext[:-1]
                else:
                    test_extension = file_ext

                if fext[0 : len(test_extension)] != test_extension:
                    file_name = ""
                    print(
                        "\n*** File given is not a "
                        + file_ext
                        + " file, try again ***\n"
                    )
                    continue
            break
        else:
            file_name = ""
            print("\n*** File given does exist, try again ***\n")

    print("\n" + file_name.split("/")[-1] + " opened")
    return open(file_name, open_mode)


def checkequal(iterator):
    try:
        iterator = iter(iterator)
        first = next(iterator)
        return all(first == rest for rest in iterator)
    except StopIteration:
        return True
