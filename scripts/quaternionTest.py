import rospy
import numpy as np
from tf.transformations import quaternion_matrix, quaternion_multiply, quaternion_inverse

def apply_rotation(q, vector):
    # Convert quaternion to transformation matrix
    matrix = quaternion_matrix(q)

    # Convert the 3D vector to a 4D homogeneous vector by appending a 1
    homogeneous_vector = np.append(vector, 1)

    # Apply the transformation matrix to the homogeneous vector
    rotated_homogeneous_vector = np.dot(matrix, homogeneous_vector)

    # Convert the rotated homogeneous vector back to a 3D vector by removing the last element
    rotated_vector = rotated_homogeneous_vector[:3]

    return rotated_vector
if __name__ == '__main__':
    # Define the quaternion (x, y, z, w) and the vector (x, y, z)
    quaternion = [ 0, 0, 0.7068252, 0.7073883 ]
    vector = [1.0, 0.0, 0.0]

    # Apply the rotation
    rotated_vector = apply_rotation(quaternion, vector)

    print("Original vector:", vector)
    print("Rotated vector:", rotated_vector)
