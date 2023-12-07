import numpy as np

class GeometryMathMethods():
    
    @classmethod
    def transformation_matrix(cls, x, y, z, q):
    # Координаты и кватернион преобразуются в матрицы
        translation_matrix = np.array([[1, 0, 0, x],
                                    [0, 1, 0, y],
                                    [0, 0, 1, z],
                                    [0, 0, 0, 1]])
        
        rotation_matrix = np.array([[1 - 2 * (q[1]**2 + q[2]**2), 2 * (q[0] * q[1] - q[2] * q[3]), 2 * (q[0] * q[2] + q[1] * q[3]), 0],
                                    [2 * (q[0] * q[1] + q[3] * q[2]), 1 - 2 * (q[0]**2 + q[2]**2), 2 * (q[1] * q[2] - q[0] * q[3]), 0],
                                    [2 * (q[0] * q[2] - q[3] * q[1]), 2 * (q[1] * q[2] + q[3] * q[0]), 1 - 2 * (q[0]**2 + q[1]**2), 0],
                                    [0, 0, 0, 1]])
        
        # Умножение матриц для получения матрицы перехода
        transformation_matrix = np.dot(translation_matrix, rotation_matrix)
        
        return transformation_matrix
    
    @classmethod
    def retrieve_coordinates_and_quaternion(cls, transformation_matrix):
    # Извлечение координат из матрицы перехода
        x = transformation_matrix[0, 3]
        y = transformation_matrix[1, 3]
        z = transformation_matrix[2, 3]
        
        # Извлечение кватерниона из матрицы перехода
        trace = np.trace(transformation_matrix[:3, :3])
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1)
            qw = 0.25 / s
            qx = (transformation_matrix[2, 1] - transformation_matrix[1, 2]) * s
            qy = (transformation_matrix[0, 2] - transformation_matrix[2, 0]) * s
            qz = (transformation_matrix[1, 0] - transformation_matrix[0, 1]) * s
        else:
            if transformation_matrix[0, 0] > transformation_matrix[1, 1] and transformation_matrix[0, 0] > transformation_matrix[2, 2]:
                s = 2.0 * np.sqrt(1.0 + transformation_matrix[0, 0] - transformation_matrix[1, 1] - transformation_matrix[2, 2])
                qw = (transformation_matrix[2, 1] - transformation_matrix[1, 2]) / s
                qx = 0.25 * s
                qy = (transformation_matrix[0, 1] + transformation_matrix[1, 0]) / s
                qz = (transformation_matrix[0, 2] + transformation_matrix[2, 0]) / s
            elif transformation_matrix[1, 1] > transformation_matrix[2, 2]:
                s = 2.0 * np.sqrt(1.0 + transformation_matrix[1, 1] - transformation_matrix[0, 0] - transformation_matrix[2, 2])
                qw = (transformation_matrix[0, 2] - transformation_matrix[2, 0]) / s
                qx = (transformation_matrix[0, 1] + transformation_matrix[1, 0]) / s
                qy = 0.25 * s
                qz = (transformation_matrix[1, 2] + transformation_matrix[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + transformation_matrix[2, 2] - transformation_matrix[0, 0] - transformation_matrix[1, 1])
                qw = (transformation_matrix[1, 0] - transformation_matrix[0, 1]) / s
                qx = (transformation_matrix[0, 2] + transformation_matrix[2, 0]) / s
                qy = (transformation_matrix[1, 2] + transformation_matrix[2, 1]) / s
                qz = 0.25 * s
        
        return x, y, z, [qw, qx, qy, qz]
    