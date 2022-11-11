from sympy import *
Φ = symbols('Φ')
Θ = symbols('Θ')
Ψ = symbols('Ψ')
Rx = Matrix([[1, 0, 0],
            [0, cos(Φ), -sin(Φ)],
            [0, sin(Φ), cos(Φ)]])
Ry = Matrix([[cos(Θ), 0, sin(Θ)],
            [0, 1, 0],
            [-sin(Θ), 0, cos(Θ)]])
Rz = Matrix([[cos(Ψ), sin(Ψ), 0],
            [-sin(Ψ), cos(Ψ), 0],
            [0, 0, 1]])

R = Rx*Ry*Rz
pprint(R)
