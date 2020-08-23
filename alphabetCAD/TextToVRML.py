''' Program to create alphabet objects as VRML97 objects to
    be imported into Webots.

    Relies on a FreeCAD installation and local installed fonts,
    so you will likely need to set these paths below.

    Author: Evan Thomas
    Date: 21/05/2020
    Contact: evant1@student.unimelb.edu.au

'''

import functools
import os
import sys
import string

FREECADPATH = "/usr/lib/freecad/lib"
FONTFOLDER = "/usr/share/fonts/TTF/"
FONTNAME = "DejaVuSans-Bold.ttf"
OUTFOLDER = "output"

sys.path.append(FREECADPATH)
from FreeCAD import Part, Base, MeshPart, Rotation, Placement


def render_string(input_string, out_file):
    ''' Following function adapted from
        https://forum.freecadweb.org/viewtopic.php?t=31885

        Uses a character input (can be a string) and forms
        a VRML97 CAD representation using a locally installed
        font.
    '''

    def fusePartList(l):
        return functools.reduce(lambda a, b: a.fuse(b), l)

    string = Part.makeWireString(input_string, FONTFOLDER, FONTNAME, 3, 0.25)
    t = fusePartList([Part.Face(c) for c in string])
    t = t.extrude(Base.Vector(0, 0, 1))
    t = MeshPart.meshFromShape(t)

    ''' Rotate 90° so characters lay on ground.
        Assumption here is that the characters are created upright.
    '''
    rotation_vector = Rotation(Base.Vector(1,0,0), -90)
    centre_vector = Base.Vector(0, 0, 0)
    position_vector = t.Placement.Base
    new_placement = Placement(position_vector, rotation_vector, centre_vector)
    print(t.Placement)
    t.Placement = new_placement
    print(t.Placement)
    t.write(out_file)


def create_out_folder(folder_path, folder_name):
    try:
        os.mkdir(os.path.join(folder_path, folder_name))
    except OSError as e:
        if e.errno == 17:
            print("Output folder '{}' already created,"
                  "will using this.".format(folder_name))
        else:
            print("OSError: {}".format(e.errno))
            sys.exit()
    else:
        print("Created folder: {}".format(folder_name))


def main():
    create_out_folder(os.getcwd(), OUTFOLDER)

    for character in list(string.ascii_lowercase):
        input_string = character
        filename = str(input_string) + ".wrl"
        outfile_path = os.path.join(os.getcwd(), OUTFOLDER, filename)
        render_string(input_string, outfile_path)
        print("\nCreated string '{}'".format(input_string))


if __name__ == "__main__":
    main()
