from setuptools import setup, find_packages
import os

package_name = "emg_device_left"

def existing_files(paths):
    return [p for p in paths if os.path.isfile(p)]


model_candidates = existing_files([
    os.path.join(package_name, "models", "re_model_2.joblib"),
])

data_files = [
    ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
    (f"share/{package_name}", ["package.xml"]),
    (f"share/{package_name}/launch", [
        "launch/emg_signal_left_launch.py",
        "launch/emg_full_pipeline_left_launch.py",
    ]),
]

if os.path.isfile("LICENSE"):
    data_files.append((f"share/{package_name}/LICENSE", ["LICENSE"]))

if model_candidates:
    data_files.append((f"share/{package_name}/models", model_candidates))
else:
    print(f"[setup.py] WARNING: no model files found under {package_name}/models")

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gkim451",
    maintainer_email="tj6774@gmail.com",
    description="ROS 2 EMG driver + real-time gesture classification.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "emg_node_left = emg_device_left.emg_node_left:main",
            "emg_classifier_node_left = emg_device_left.emg_classifier_node:main",
        ],
    },
)
