# --------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.
# --------------------------------------------------------------------------------------------

import os,sys

# -----------------------------------------------------------------------------
#
def initialize():
    global appVersion           # application version
    global fVerbose             # verbose console output
    global application          # application object
    global cwd                  # current working directory
    global numHttpServers
    global threadIOLoop

    if (os.name == 'nt'):
        os.system('color')  # needed on windows platforms to support terminal colors

    appVersion = 'v1.00.0102'
    fVerbose = False
    application = None

    cwd = os.getcwd()

    # If we're in the "src" directory, use the parent directory as the base path
    # for files to make specifying paths a little easier for the user
    if (cwd[-4:] == '\\src') or (cwd[-4:] == '/src'):
        cwd = cwd[0:-4]
 
    numHttpServers = 0
    threadIOLoop = None

