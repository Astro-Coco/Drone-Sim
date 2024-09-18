import setuptools

"""with open("README.md", "r", encoding="utf-8") as fhand:
    long_description = fhand.read()"""

setuptools.setup(
    name="Drone-Sim",
    version="1.0.0",
    author=("Colin Rousseau", "François Drapeau"),
    description=("Flight sim of a drone"),
    #long_description=long_description,
    #long_description_content_type="text/markdown",
    url="https://github.com/Astro-Coco/Drone-Sim",
    project_urls={
        "Bug Tracker": "https://github.com/Astro-Coco/Drone-Sim",
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    #install_requires=["requests"],
    packages=setuptools.find_packages(),
    python_requires=">=3.12.0",
    entry_points={
        "console_scripts": [
            "sim = drone_dir.sim_material.cli:main",
        ]
    }
)