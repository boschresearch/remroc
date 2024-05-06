
# Copyright (c) 2024 - for information on the respective copyright owner
# see the NOTICE file or the repository https://github.com/boschresearch/remroc/.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.



from setuptools import find_packages, setup
from glob import glob
from setuptools import setup
from pathlib import Path

package_name = 'remroc_world_generation'


def data_file_list(target_path):
    return_list = []
    path_list = [x.as_posix() for x in Path(target_path).rglob('*.*')]
    for x in path_list:
        return_list.append((Path('.').joinpath('share', package_name, x).parent.as_posix(), [x]))
    return return_list

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])] +
        data_file_list('launch') +
        data_file_list('worlds') +
        data_file_list('params'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hee7rng',
    maintainer_email='lukas.heuer@de.bosch.com',
    description='Package which contains code to generate human trajectories',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_ready_listener = remroc_world_generation.planner_ready_listener:main',
            'compute_path_to_pose_client = remroc_world_generation.compute_path_to_pose_client:main',
            'world_generation_node = remroc_world_generation.world_generation_node:main'
        ],
    },
)