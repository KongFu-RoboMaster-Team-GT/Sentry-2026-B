"""Exercise actual suspend/resume machine code from the built UPPER.axf.

USB clock/SCB registers are simulated memory, not physical USB or a host.
Requires the same Unicorn/pyelftools environment as run_protocol.py.
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'MDK-ARM/build/test_deps'))
from elftools.elf.elffile import ELFFile
from elftools.dwarf.dwarf_expr import DWARFExprParser
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC, UC_CPU_ARM_CORTEX_M4


def run(image_path):
    with image_path.open('rb') as stream:
        elf = ELFFile(stream)
        symbols = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
        dwarf = elf.get_dwarf_info()
        variables = {}
        for cu in dwarf.iter_CUs():
            for die in cu.iter_DIEs():
                if die.tag == 'DW_TAG_variable' and 'DW_AT_name' in die.attributes and 'DW_AT_type' in die.attributes:
                    name = die.attributes['DW_AT_name'].value.decode()
                    variables.setdefault(name, die.get_DIE_from_attribute('DW_AT_type'))

        def unwrap(die):
            while die.tag in ('DW_TAG_typedef', 'DW_TAG_const_type', 'DW_TAG_volatile_type'):
                die = die.get_DIE_from_attribute('DW_AT_type')
            return die

        def field(variable, path):
            die, offset = variables[variable], 0
            for name in path.split('.'):
                die = unwrap(die)
                member = next(child for child in die.iter_children()
                              if child.tag == 'DW_TAG_member' and child.attributes.get('DW_AT_name')
                              and child.attributes['DW_AT_name'].value.decode() == name)
                location = member.attributes['DW_AT_data_member_location'].value
                if not isinstance(location, int):
                    ops = DWARFExprParser(dwarf.structs).parse_expr(location)
                    assert len(ops) == 1 and ops[0].op_name == 'DW_OP_plus_uconst'
                    location = ops[0].args[0]
                offset += location
                die = member.get_DIE_from_attribute('DW_AT_type')
            die = unwrap(die)
            size = (elf.elfclass // 8 if die.tag == 'DW_TAG_pointer_type'
                    else die.attributes['DW_AT_byte_size'].value)
            return symbols[variable] + offset, size

        cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        cpu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
        for base, size in ((0x08000000, 0x100000), (0x20000000, 0x20000),
                           (0x10000000, 0x10000), (0x50000000, 0x1000), (0xE000E000, 0x1000)):
            cpu.mem_map(base, size)
        for section in elf.iter_sections():
            if section['sh_flags'] & 2 and section['sh_type'] != 'SHT_NOBITS':
                cpu.mem_write(section['sh_addr'], section.data())

        def write(address, value, size=4):
            cpu.mem_write(address, value.to_bytes(size, 'little'))

        def read(address, size=4):
            return int.from_bytes(cpu.mem_read(address, size), 'little')

        def set_field(variable, path, value):
            address, size = field(variable, path)
            write(address, value, size)

        def get_field(variable, path):
            return read(*field(variable, path))

        def call(name):
            cpu.reg_write(UC_ARM_REG_SP, 0x2001FF00)
            cpu.reg_write(UC_ARM_REG_R0, symbols['hpcd_USB_OTG_FS'])
            cpu.reg_write(UC_ARM_REG_LR, 0x2001FFF1)
            cpu.emu_start(symbols[name] | 1, 0x2001FFF0, count=10000)
            assert cpu.reg_read(UC_ARM_REG_PC) == 0x2001FFF0, f'{name} did not return'

        set_field('hpcd_USB_OTG_FS', 'Instance', 0x50000000)
        set_field('hpcd_USB_OTG_FS', 'pData', symbols['hUsbDeviceFS'])
        set_field('hpcd_USB_OTG_FS', 'Init.low_power_enable', 0)
        pcgcctl, scr = 0x50000E00, 0xE000ED10
        checks = 0
        # Real callbacks must keep PHY running in this project's non-STOP mode.
        # Test both pre-enumeration idle and an already-configured connection.
        for state in (1, 2, 3):
            for cycle in range(10):
                set_field('hUsbDeviceFS', 'dev_state', state)
                write(pcgcctl, 1)  # Also recover a previously-gated PHY clock.
                write(scr, 0)
                call('HAL_PCD_SuspendCallback')
                assert get_field('hUsbDeviceFS', 'dev_state') == 4
                assert get_field('hUsbDeviceFS', 'dev_old_state') == state
                assert read(pcgcctl) & 1 == 0 and read(scr) == 0
                call('HAL_PCD_SuspendCallback')
                assert get_field('hUsbDeviceFS', 'dev_old_state') == state
                write(pcgcctl, 1)
                call('HAL_PCD_ResumeCallback')
                assert read(pcgcctl) & 1 == 0
                assert get_field('hUsbDeviceFS', 'dev_state') == state
                checks += 6
        # Preserve the existing opt-in low-power branch; it is disabled in UPPER.
        set_field('hpcd_USB_OTG_FS', 'Init.low_power_enable', 1)
        write(pcgcctl, 0)
        call('HAL_PCD_SuspendCallback')
        assert read(pcgcctl) & 1 and read(scr) & 6 == 6
        print(f'{image_path.name}: {checks + 1} USB power checks passed (actual firmware callbacks)')


if __name__ == '__main__':
    run(Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'MDK-ARM/build/UPPER/UPPER.axf')
