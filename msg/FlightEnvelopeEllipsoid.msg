# Describe an ellipsoid flight envelope
Header header
# Box constraints
float64 Va_max
float64 Va_min
float64 gamma_max
float64 gamma_min
float64 R_min
# Parameters for the definition of the Flight Envelope ellipsoid
# Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz + J = 0
# where x:Va, y: gamma, z: psi_dot
float64 el_A
float64 el_B
float64 el_C
float64 el_D
float64 el_E
float64 el_F
float64 el_G
float64 el_H
float64 el_I
float64 el_J