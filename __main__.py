# import roboflow package with its necessary modules. This is needed for the absolute import to work
import roboflow.motionPlanning
import roboflow.slam
import roboflow.tests
import sys

def main(testCase):
    if testCase == 'motion':
        # edekaMotion.py is a full script. This import will simply run it!
        import roboflow.tests.edekaMotion
    elif testCase == 'slam':
        # edekaSlam.py is a full script. This import will simply run it!
        import roboflow.tests.edekaSlam
    else:
        sys.exit('case is not defined!')

if __name__ == "__main__":
    print('Which test case (available: motion, slam)?')
    testCase = input()
    main(testCase)