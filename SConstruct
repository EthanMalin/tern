import os

env = Environment()

env['ROOT'] = GetLaunchDir()
env['SRC'] = os.path.join(env['ROOT'], 'src')
env['TEST_SRC'] = os.path.join(env['SRC'], 'tests')
env['OBJ'] = os.path.join(env['ROOT'], 'build', 'obj')
env['BIN'] = os.path.join(env['ROOT'], 'build', 'bin')

# build quaternion library
libEnv = env.Clone()
tern_obj = libEnv.Object(os.path.join(env['OBJ'], 'tern.o'), os.path.join(env['SRC'], 'tern.c'))
tern = libEnv.Library(os.path.join(env['BIN'], 'tern'), tern_obj)

# build test app
testEnv = env.Clone()
testEnv.Append(CPPPATH=[os.path.join(env['SRC'])])
testEnv.Append(LIBS=[tern])
test_obj = testEnv.Object(os.path.join(env['OBJ'], 'tests', 'tests.o'), os.path.join(env['TEST_SRC'], 'tests.c'))
test_main_obj = testEnv.Object(os.path.join(env['OBJ'], 'tests', 'main.o'), os.path.join(env['TEST_SRC'], 'main.c'))
test_exe = testEnv.Program(os.path.join(env['BIN'], 'tests', 'tern_test'), [test_main_obj, test_obj])
