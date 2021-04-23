Import('env', 'envCython', 'arch', 'QCOM_REPLAY')

import shutil

cereal_dir = Dir('.')
gen_dir = Dir('gen')
messaging_dir = Dir('messaging')

# Build cereal

schema_files = ['log.capnp', 'state.capnp']
env.Command(["gen/c/include/c++.capnp.h", "gen/c/include/java.capnp.h"], [], "mkdir -p " + gen_dir.path + "/c/include && touch $TARGETS")
env.Command([f'gen/cpp/{s}.c++' for s in schema_files] + [f'gen/cpp/{s}.h' for s in schema_files],
            schema_files,
            f"capnpc --src-prefix={cereal_dir.path} $SOURCES -o c++:{gen_dir.path}/cpp/")

if shutil.which('capnpc-java'):
  env.Command(['gen/java/State.java', 'gen/java/Log.java'],
              schema_files,
              f"capnpc $SOURCES --src-prefix={cereal_dir.path} -o java:{gen_dir.path}/java/")

# TODO: remove non shared cereal and messaging
cereal_objects = env.SharedObject([f'gen/cpp/{s}.c++' for s in schema_files])

env.Library('cereal', cereal_objects)
env.SharedLibrary('cereal_shared', cereal_objects)

# Build messaging

services_h = env.Command(['services.h'], ['services.py'], 'python3 ' + cereal_dir.path + '/services.py > $TARGET')

messaging_objects = env.SharedObject([
  'messaging/messaging.cc',
  'messaging/impl_zmq.cc',
  'messaging/socketmaster.cc',
])

messaging_lib = env.Library('messaging', messaging_objects)
Depends('messaging/impl_zmq.cc', services_h)


envCython.Program('messaging/messaging_pyx.so', 'messaging/messaging_pyx.pyx', LIBS=envCython["LIBS"]+[messaging_lib, "zmq"])


libs = envCython["LIBS"]+["OpenCL", "zmq", messaging_lib]
if arch == "Darwin":
  del libs[libs.index('OpenCL')]
  envCython['FRAMEWORKS'] += ['OpenCL']


