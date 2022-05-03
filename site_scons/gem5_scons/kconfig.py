# Copyright 2022 Google, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os

from . import error
import kconfiglib

def _prep_env(env, base_kconfig, config_path=None):
    kconfig_env = env.Clone()
    for key, val in kconfig_env['CONF'].items():
        if isinstance(val, bool):
            val = 'y' if val else 'n'
        kconfig_env['ENV'][key] = val
    kconfig_env['ENV']['CONFIG_'] = ''
    if config_path:
        kconfig_env['ENV']['KCONFIG_CONFIG'] = config_path

    ext = env.Dir('#ext')
    kconfiglib_dir = ext.Dir('Kconfiglib')
    defconfig_py = kconfiglib_dir.File('defconfig.py')
    menuconfig_py = kconfiglib_dir.File('menuconfig.py')

    kconfig_env['DEFCONFIG_PY'] = defconfig_py
    kconfig_env['MENUCONFIG_PY'] = menuconfig_py
    kconfig_env['BASE_KCONFIG'] = base_kconfig
    return kconfig_env

def defconfig(env, base_kconfig, config_in, config_out):
    kconfig_env = _prep_env(env, base_kconfig, config_out)
    kconfig_env['CONFIG_IN'] = config_in
    if kconfig_env.Execute('"${DEFCONFIG_PY}" --kconfig "${BASE_KCONFIG}" '
            '"${CONFIG_IN}"') != 0:
        error("Failed to run defconfig")

def menuconfig(env, base_kconfig, config_path, main_menu_text,
        style='aquatic'):
    kconfig_env = _prep_env(env, base_kconfig, config_path)
    kconfig_env['ENV']['MENUCONFIG_STYLE'] = style
    kconfig_env['ENV']['MAIN_MENU_TEXT'] = main_menu_text
    if kconfig_env.Execute('"${MENUCONFIG_PY}" "${BASE_KCONFIG}"') != 0:
        error("Failed to run menuconfig")

def update_env(env, base_kconfig, config_path):
    kconfig_env = _prep_env(env, base_kconfig, config_path)

    saved_env = os.environ
    os.environ.update({key: str(val) for key, val in
            kconfig_env['ENV'].items()})
    kconfig = kconfiglib.Kconfig(filename=base_kconfig)
    os.environ = saved_env

    kconfig.load_config(config_path)
    for sym in kconfig.unique_defined_syms:
        val = sym.str_value
        if sym.type == kconfiglib.BOOL:
            env['CONF'][sym.name] = True if val == 'y' else False
        elif sym.type == kconfiglib.TRISTATE:
            warning('No way to configure modules for now')
            env['CONF'][sym.name] = True if val == 'y' else False
        elif sym.type == kconfiglib.INT:
            if not val:
                val = '0'
            env['CONF'][sym.name] = int(val, 0)
        elif sym.type == kconfiglib.HEX:
            if not val:
                val = '0'
            env['CONF'][sym.name] = int(val, 16)
        elif sym.type == kconfiglib.STRING:
            env['CONF'][sym.name] = val
        elif sym.type == kconfiglib.UNKNOWN:
            warning(f'Config symbol "{sym.name}" has unknown type')
            env['CONF'][sym.name] = val
        else:
            type_name = kconfiglib.TYPE_TO_STR[sym.type]
            error(f'Unrecognized symbol type {type_name}')
