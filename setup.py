import os
from glob import glob
from setuptools import setup
from setuptools import find_packages


package_name = 'maila_description'
pkg_path = os.path.dirname(os.path.realpath(__file__))


def copy_all(destination, source):
    data_files = []
    folder_lists = [source]
    while len(folder_lists) > 0:
        file_lists = []
        curr_folder = folder_lists[0]
        curr_folder_path = os.path.join(pkg_path, curr_folder)
        for f in os.listdir(curr_folder_path):
            new_element = os.path.join(curr_folder, f)
            new_element_path = os.path.join(pkg_path, new_element)
            if os.path.isdir(new_element_path):
                folder_lists.append(new_element)
            if os.path.isfile(new_element_path):
                file_lists.append(new_element)
        if len(file_lists) > 0:
            data_files.append(
                (os.path.join(destination, curr_folder), file_lists))
        folder_lists.pop(0)
    return data_files


data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    (os.path.join('share', package_name), ['package.xml']),
    (os.path.join('share', package_name), glob('launch/*.py'))]

data_files += copy_all(os.path.join('share', package_name), 'urdf')

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='snail',
    maintainer_email='valerio.magnago@hotmail.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_state_publisher = maila_description.dummy_state_publisher:main'
        ],
    },
)
