

import cmath
import numpy as np

# Poles / obstacle markers
zeta = [complex(2, 3), complex(6, 7), complex(-5, 10), complex(5, -6)]

# Hard code the case here
BL = complex(-20, -20)
TR = complex(20, 20)
N = 4
a = N // 2
b = (N - 1) - a

# Output homotopy classes from an extensive run of the algorithm
homot_classes = [complex(a,b) for (a,b) in
    [(-19.80, -61.57),     
    (0.17, 1.69),      
    (69.44, -66.37),      
    (0.17, 7.98), 
    (89.42, -3.11),    
    (-89.07, 12.78),      
    (20.15, 71.24),  
    (-69.09, 76.04),       
    (49.46, -135.92),     
    (-109.05, -56.77), 
    (-89.07, 6.49),       
    (69.44, -72.66),      
    (138.70, -140.72), 
    (-39.78, -124.84),    
    (20.15, 64.96),   
    (158.68, -71.17),      
    (-69.09, 82.33),      
    (89.42, 3.18),   
    (158.68, -77.46),     
    (178.66, -7.91),      
    (49.46, -129.64),  
    (-158.33, 87.13),     
    (-19.80, -55.29), 
    (-178.31, 17.58),      
    (109.39, 60.16),      
    (-109.05, -50.49), 
    (-39.78, -131.12),   
    (-19.80, -67.86),     
    (-49.11, 145.59),  
    (109.39, 66.44),      
    (89.42, -9.39),   
    (-138.35, 150.39),
    (40.13, 134.51),      
    (20.15, 77.53)]
]

# Tolerance in comparing
float_tol = 0.1


def f_0(z):
    return ((z - BL)**a) * ((z - TR)**b)

def compute_residues():
    A = []
    for i in range(N):
        prod = complex(1.0, 0.0)
        for j in range(N):
            if (j==i):
                continue
            prod *= zeta[i] - zeta[j]
        A.append(f_0(zeta[i]) / prod)
    return A


def identify_residue_multiples(A):
    '''
    A contains pre-computed residue values, and we attempt to find the duplicates in homot_classes by the residue theorem,
    comparing each element of A times 2*pi*i, plus an iterator, to each value of the list homot_classes.
    The list C contains the undesired duplicates, which are added only when not already present in C
    We finally compute the list of the remaining unique values
    '''
    # Keep list C of duplicates and running count of total duplicates
    C = []
    total_duplicates = 0
    for i in range(len(homot_classes)):
        homot_class = homot_classes[i]
        homot_duplicates = 0 # classes that correspond to homot_classes[i] with some extra loops
        for j in range(len(homot_classes)):
            other_class = homot_classes[j]
            if (i==j):
                continue
            # Check if any (other_class - (2*pi*i) * a) falls within tolerance of homot_class, since we want to eliminate the class with loops
            pole_number = -1
            for k in range(len(A)):
                if abs((other_class - (2*np.pi*complex(0.0,1.0)) * A[k]) - homot_class) <= float_tol:
                    pole_number = k
            if pole_number == -1:
                continue
            # Add only if the other class was not already identified as duplicate
            if other_class in C: 
                continue
            # Record detection of duplicate
            print(str(homot_class) + " has homotopic duplicate " + str(other_class) + " due to pole number " + str(pole_number))
            homot_duplicates += 1
            C.append(other_class)
        print("Found " + str(homot_duplicates) + " duplicates for homotopic class " + str(homot_class))
        total_duplicates += homot_duplicates 
    print("Found total duplicates " + str(total_duplicates) + " out of " + str(len(homot_classes)) + " total homotopic classes")
    # Make list of unique homotopic classes
    U = []
    for homot_class in homot_classes:
        if homot_class not in C:
            U.append(homot_class)
    print("Unique homotopic classes " + str(U))
    return U



def main():
    
    A = compute_residues()
    identify_residue_multiples(A)



if __name__ == '__main__':
    main()


