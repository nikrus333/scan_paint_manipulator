import os
import numpy as np
from dataclasses import dataclass
from math import radians 
import gcodeparser

@dataclass
class PoseGcode():
    x: float
    y: float
    z: float
    nx: float
    ny: float
    nz: float
    
@dataclass
class MOVL():
    type: str
    pose: PoseGcode
    
@dataclass
class MOVC():
    type: str
    start: PoseGcode
    middle: PoseGcode
    end: PoseGcode

class Gcode():
    def __init__(self, path: str) -> None:
        self._path = path
        gcode_line = self.GcodeFromPath(self._path)
        self._command = self.GcodeToArr(gcode_line)
        pass
    
    def get_commands(self) -> list[MOVC | MOVL]:
        return self._command
    
    def get_type(self) -> str:
        return self._command
    
    def get_path(self) -> str:
        return self._path
    
    @staticmethod
    def GcodeToArr(gcode_line: list[str]) -> list[MOVC | MOVL]:
        list_command = []
        for line in gcode_line:
            # Разделяем строку по символу пробела или символу перед значением
            line = line.split(" ")
            command = None
            if line[0] == "MOVL":
                pose = PoseGcode(
                    x = float(line[1][1:]),
                    y = float(line[2][1:]),
                    z = float(line[3][1:]),
                    nx = float(line[4][1:]),
                    ny = float(line[5][1:]),
                    nz = float(line[6][1:]),
                )
                command = MOVL(type="MOVL", pose=pose)
                
            if line[0] == "MOVC":
                start = PoseGcode(
                    x = float(line[1][2:]),
                    y = float(line[2][2:]),
                    z = float(line[3][2:]),
                    nx = float(line[4][2:]),
                    ny = float(line[5][2:]),
                    nz = float(line[6][2:]),
                    )
                middle = PoseGcode(
                    x = float(line[7][2:]),
                    y = float(line[8][2:]),
                    z = float(line[9][2:]),
                    nx = float(line[10][2:]),
                    ny = float(line[11][2:]),
                    nz = float(line[12][2:]),
                )
                end = PoseGcode(
                    x = float(line[13][2:]),
                    y = float(line[14][2:]),
                    z = float(line[15][2:]),
                    nx = float(line[16][2:]),
                    ny = float(line[17][2:]),
                    nz = float(line[18][2:]),
                )
                command = MOVC(type=line[0], start=start, middle=middle, end=end)
            if command != None:
                list_command.append(command)
        return list_command
    
    @staticmethod
    def GcodeFromPath(path: str) -> str:
        f = open(path, "r")
        lines = f.readlines()
        gcode_line = []
        for line in lines:
            gcode_line.append(line.replace('\n', ''))
        return gcode_line
    
    @staticmethod
    def GcodeToArray(gcode_line: str, scale_ = 0.001):
        """
        array -> "G1 X0.0 Y0.0\\nG1 X1.0 Y0.0\\n"
        """
        points = []
        currentPath = {"type": "", "points":[]}
        scale = scale_

        lines = gcodeparser.GcodeParser(gcode_line).lines
        for line in lines:
            if (line.command == ('G', 21)):
                print("Metric system")
            elif (line.command == ('G', 90)):
                print("Absolyte coordinate")
            elif (line.command == ('G', 0)):
                if (currentPath["type"] != "L"):
                    points.append(currentPath)
                    currentPath = {"type": "L", "points":[]}
            
                currentPath["points"].append([(line.get_param('X')*scale), (line.get_param('Y')*scale), (line.get_param('Z')*scale+0.185)] + [radians(180.0), radians(.0), radians(90.0)])
        
            elif (line.command == ('G', 1)):
                if (currentPath["type"] != 'L'):
                    points.append(currentPath)
                    currentPath = {"type": 'L', "points":[]}
                        
                currentPath["points"].append([(line.get_param('X')*scale), (line.get_param('Y')*scale), (line.get_param('Z')*scale+0.185)] + ([radians(180.0), radians(.0), radians(90.0)]))
        points.append(currentPath)
        return points

    
if __name__ == "__main__":
    path = os.getcwd() + "/paint_lidar/resource/gcode/TestC.gcode"
    g = Gcode(path)
    command = g.get_commands()
    print(command)
    pass
    
    
