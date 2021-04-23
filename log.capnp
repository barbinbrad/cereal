using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

using Java = import "./include/java.capnp";
$Java.package("have.some.delicious.cereal");
$Java.outerClassname("Log");

using State = import "state.capnp";

@0xf3b1f17e25a4285b;

const logVersion :Int32 = 1;

struct Map(Key, Value) {
  entries @0 :List(Entry);
  struct Entry {
    key @0 :Key;
    value @1 :Value;
  }
}

struct Event {
  logMonoTime @0 :UInt64;  # nanoseconds
  valid @1 :Bool = true;

}