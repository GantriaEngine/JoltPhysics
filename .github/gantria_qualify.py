"""Bounded native qualification for the independently maintained public patchset."""
import argparse
import datetime
import json
import pathlib
import platform
import subprocess
import time

Parser = argparse.ArgumentParser()
Parser.add_argument('--mode', choices=['Release', 'Sanitizers'], default='Release')
Args = Parser.parse_args()
Root = pathlib.Path(__file__).resolve().parents[1]
Evidence = Root / 'qualification-evidence'
Evidence.mkdir(exist_ok=True)
Build = Root / ('build-gantria-' + Args.mode.lower())
Receipts = []

def Run(Name, Command):
    Start = time.monotonic()
    with (Evidence / (Name + '.log')).open('w', encoding='utf-8') as Log:
        Process = subprocess.run(Command, cwd=Root, stdout=Log, stderr=subprocess.STDOUT, timeout=1200)
    Receipts.append({'Name': Name, 'Command': Command, 'ExitCode': Process.returncode,
                     'Seconds': time.monotonic() - Start})
    (Evidence / 'Receipts.json').write_text(json.dumps(Receipts, indent=2) + '\n', encoding='utf-8')
    print(f'[Physics:Qualification] {Name}: exit {Process.returncode}', flush=True)
    if Process.returncode:
        print((Evidence / (Name + '.log')).read_text(encoding='utf-8')[-12000:], flush=True)
    Process.check_returncode()

Metadata = {'Source': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=Root, text=True).strip(),
            'UpstreamBase': 'e77f175595e64cb44218cc9d9d56fc365ad0e36a',
            'Platform': platform.platform(), 'Mode': Args.mode,
            'Utc': datetime.datetime.now(datetime.timezone.utc).isoformat()}
(Evidence / 'Source.json').write_text(json.dumps(Metadata, indent=2) + '\n', encoding='utf-8')
Options = ['cmake', '-S', 'Build', '-B', str(Build), '-G', 'Ninja',
           '-DCMAKE_BUILD_TYPE=Release', '-DTARGET_UNIT_TESTS=ON',
           '-DTARGET_HELLO_WORLD=OFF', '-DTARGET_PERFORMANCE_TEST=OFF',
           '-DTARGET_SAMPLES=OFF', '-DTARGET_VIEWER=OFF',
           '-DJPH_USE_DX12=OFF', '-DJPH_USE_VK=OFF', '-DJPH_USE_MTL=OFF',
           '-DJPH_USE_CPU_COMPUTE=OFF', '-DINTERPROCEDURAL_OPTIMIZATION=OFF',
           '-DGENERATE_DEBUG_SYMBOLS=OFF', '-DENABLE_ALL_WARNINGS=OFF',
           '-DFLOATING_POINT_EXCEPTIONS_ENABLED=OFF', '-DOVERRIDE_CXX_FLAGS=OFF',
           '-DCROSS_PLATFORM_DETERMINISTIC=ON', '-DPROFILER_IN_DEBUG_AND_RELEASE=OFF',
           '-DDEBUG_RENDERER_IN_DEBUG_AND_RELEASE=OFF']
for Feature in ['SSE4_1', 'SSE4_2', 'AVX', 'AVX2', 'AVX512', 'LZCNT', 'TZCNT', 'F16C', 'FMADD']:
    Options.append('-DUSE_' + Feature + '=OFF')
if platform.system() != 'Windows':
    Options += ['-DCMAKE_CXX_COMPILER=clang++-18']
if Args.mode == 'Sanitizers':
    Options += ['-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer',
                '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined']
Run('Configure', Options)
Run('Build', ['cmake', '--build', str(Build), '--parallel', '2', '--target', 'UnitTests'])
Run('Tests', ['ctest', '--test-dir', str(Build), '--verbose', '--output-on-failure',
              '--output-junit', str(Evidence / 'JUnit.xml'), '--timeout', '600'])
