import numpy as np

# Taking a 3 * 3 matrix

np.set_printoptions(suppress=True)
A = np.array(
    [
        [
            0.0010841498085428513,
            -0.0036068758966438267,
            0.0016940004822989106
        ],
        [
            -0.0006921637377395656,
            0.001782155486780773,
            9.959878836458618e-05
        ],
        [
            -0.000582200954912589,
            0.0015527180988631096,
            0.00027412851975773674
        ]
    ]
             )
# Calculating the inverse of the matrix
print(np.linalg.inv(A))