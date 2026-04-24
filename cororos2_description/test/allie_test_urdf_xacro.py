# Copyright (c) 2024, b-robotized Group (template)
# Copyright (c) 2022 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the FZI Forschungszentrum Informatik nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Source of this file is the b-robotized Group template repository.
# Modified from tests in https://github.com/UniversalRobots/Universal_Robots_ROS2_Description
#
# Author: Lukas Sackewitz
# Author (template): Manuel Muth

import os
import shutil
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory


def test_urdf_xacro():
    description_package = "cororos2_description"
    description_file = "allie.urdf.xacro"
    description_file_path = os.path.join(get_package_share_directory(description_package), "urdf", description_file)

    (_, tmp_urdf_output_file) = tempfile.mkstemp(suffix=".urdf")

    xacro_command = f"{shutil.which('xacro')} {description_file_path} > {tmp_urdf_output_file}"
    check_urdf_command = f"{shutil.which('check_urdf')} {tmp_urdf_output_file}"

    try:
        xacro_process = subprocess.run(xacro_command, capture_output=True, shell=True, text=True)

        assert xacro_process.returncode == 0, " --- XACRO command failed ---\n" + xacro_process.stderr

        check_urdf_process = subprocess.run(check_urdf_command, capture_output=True, shell=True, text=True)

        assert check_urdf_process.returncode == 0, (
            "\n --- URDF check failed! --- \nYour xacro does not unfold into a proper urdf robot description.\n"
            + check_urdf_process.stderr
        )

    finally:
        os.remove(tmp_urdf_output_file)


if __name__ == "__main__":
    test_urdf_xacro()
