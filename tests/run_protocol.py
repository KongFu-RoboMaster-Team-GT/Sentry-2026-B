"""Compile actual protocol/routing C for Cortex-M4, execute it in Unicorn.

python -m pip install --target MDK-ARM/build/test_deps unicorn pyelftools
python tests/run_protocol.py
"""
from pathlib import Path
import struct
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / 'MDK-ARM' / 'build' / 'protocol_tests'
sys.path.insert(0, str(ROOT / 'MDK-ARM' / 'build' / 'test_deps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE, UC_HOOK_MEM_INVALID
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_PC, UC_ARM_REG_C1_C0_2, UC_ARM_REG_FPEXC, UC_CPU_ARM_CORTEX_M4

BUILD.mkdir(parents=True, exist_ok=True)
ARM = Path(r'C:\Keil_v5\ARM\ARMCC\bin')
for role, number, extra in [('SINGLE', 0, []), ('UPPER', 1, []),
                            ('UPPER_GUARD', 1, ['-DYAW_MOTION_GUARD_ENABLE=1']),
                            ('LOWER', 2, [])]:
    obj, elf = BUILD / f'{role}.o', BUILD / f'{role}.axf'
    subprocess.run([str(ARM / 'armcc.exe'), '--cpu=Cortex-M4', '--fpu=FPv4-SP', '--c99', '-O0',
                    f'-DBOARD_ROLE={number}', '-Itests/stubs', '-IUser', '-IUser/Middleware', '-IUser/Algorithm',
                    '-IUser/Heat', '-IUser/PC',
                    *extra, '-c', 'tests/protocol_arm.c', '-o', str(obj)], cwd=ROOT, check=True)
    subprocess.run([str(ARM / 'armlink.exe'), '--entry=test_main', '--ro_base=0x1000',
                    '--rw_base=0x20000000', '--no_scanlib', str(obj), '-o', str(elf)],
                   cwd=ROOT, check=True)
    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    uc.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
    uc.reg_write(UC_ARM_REG_C1_C0_2, 0xF << 20)
    uc.reg_write(UC_ARM_REG_FPEXC, 0x40000000)
    uc.mem_map(0, 0x20000)
    uc.mem_map(0x20000000, 0x20000)
    with elf.open('rb') as stream:
        image = ELFFile(stream)
        for section in image.iter_sections():
            if section['sh_flags'] & 2 and section['sh_type'] != 'SHT_NOBITS':
                uc.mem_write(section['sh_addr'], section.data())
        symbols = {s.name: s['st_value'] for s in image.get_section_by_name('.symtab').iter_symbols()}
    done = symbols['test_done'] & ~1
    stopped = [False]
    def hook(cpu, address, size, _):
        if address == done:
            stopped[0] = True
            cpu.emu_stop()
    uc.hook_add(UC_HOOK_CODE, hook)
    def invalid(cpu, access, address, size, value, _):
        raise RuntimeError(f'{role}: memory {address:#x}, PC={cpu.reg_read(UC_ARM_REG_PC):#x}, SP={cpu.reg_read(UC_ARM_REG_SP):#x}')
    uc.hook_add(UC_HOOK_MEM_INVALID, invalid)
    uc.reg_write(UC_ARM_REG_SP, 0x2001FFF0)
    try:
        uc.emu_start(symbols['test_main'] | 1, 0, count=60_000_000 if role == 'LOWER' else 10_000_000)
    except Exception:
        print(f'PC={uc.reg_read(UC_ARM_REG_PC):#x}, SP={uc.reg_read(UC_ARM_REG_SP):#x}', flush=True)
        raise
    read = lambda name: struct.unpack('<I', uc.mem_read(symbols[name], 4))[0]
    assert stopped[0], f'{role}: execution limit at {uc.reg_read(UC_ARM_REG_PC):#x}'
    assert read('test_failed_line') == 0, f'{role}: failed protocol_arm.c:{read("test_failed_line")}'
    print(f'{role}: {read("test_assertions")} assertions passed')
