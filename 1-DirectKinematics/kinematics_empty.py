import math

# Dimensions used for the PhantomX robot :
constL1 = 54.8
constL2 = 65.3
constL3 = 133
theta2Correction = 0  # A completer
theta3Correction = 0  # A completer

# Dimensions used for the simple arm simulation
# bx = 0.07
# bz = 0.25
# constL1 = 0.085
# constL2 = 0.185
# constL3 = 0.250


def computeDK(theta1, theta2, theta3, l1=constL1, l2=constL2, l3=constL3):
    # A completer
    x = 0.2
    y = 0.2
    z = 0.5

    return [x, y, z]


def computeIK(x, y, z, l1=constL1, l2=constL2, l3=constL3):
    theta1 = 0
    theta2 = 0
    theta3 = 0

    return [theta1, theta2, theta3]


def main():
    print("Testing the kinematic funtions...")
    print(
        "computeDK(0, 0, 0) = {}".format(
            computeDK(0, 0, 0, l1=constL1, l2=constL2, l3=constL3)
        )
    )


if __name__ == "__main__":
    main()
