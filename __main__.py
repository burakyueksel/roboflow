# import roboflow package with its necessary modules. This is needed for the absolute import to work
import roboflow.motionPlanning
import roboflow.tests
def main():
    # edeka.py is a full script. This import will simply run it!
    import roboflow.tests.edeka

if __name__ == "__main__":
    main()