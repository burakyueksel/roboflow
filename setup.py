import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="roboflow", # name of the package
    version="0.0.1",
    author="Burak Yueksel",
    author_email="mail.burakyuksel@gmail.com",
    description="Robotics Toolbox",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="webpage here",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)