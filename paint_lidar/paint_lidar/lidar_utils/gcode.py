import os
import numpy as np
from math import radians 
import gcodeparser

class Gcode():
    def __init__(self, path: str) -> None:
        self._path = path
        self._array = []
        gcode_line = self.GcodeFromPath(self._path)
        array_ = self.GcodeToArray(gcode_line)
        for move in array_:
            if move['type'] == "L":
                for point in move['points']:
                    self._array.append(point)
        pass
    
    def get_array(self) -> np.ndarray:
        return self._array
    
    def get_path(self) -> str:
        return self._path
    
    @staticmethod
    def GcodeFromPath(path: str) -> str:
        f = open(path, "r")
        lines = f.readlines()
        gcode_line = ""
        for line in lines:
            gcode_line += line
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
    path = os.getcwd() + "/paint_lidar/resource/gcode/test_gcode.gcode"
    g = Gcode(path)
    print(g.get_array())
    pass
    
    
