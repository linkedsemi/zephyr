.. zephyr:code-sample:: posix-dup
   :name: dup()

   Verify the POSIX ``dup()`` implementation, including refcounted close
   semantics for duplicated descriptors.

Overview
********

This sample exercises the POSIX ``dup()`` semantics that matter for the fdtable
refcounted-close implementation:

- a duplicated descriptor refers to the same underlying object (an
  ``eventfd``);
- ``close()`` releases the object only when the last descriptor referring to
  it is closed, in either order (close the original first, or close the
  duplicate first);
- the duplicated descriptor inherits the object's file status flags
  (``O_NONBLOCK``);
- multiple ``dup()``s of one descriptor all share the object, and closing any
  one of them leaves the others functional;
- invalid descriptors (``-1``, an already closed fd, or an out-of-range value
  such as ``INT_MAX``) fail with ``EBADF``.

An ``eventfd`` is used instead of a regular file so the sample runs without a
mounted filesystem; none of the semantics above depend on one.

Known limitation
****************

The file offset lives in the per-descriptor ``fdtable`` entry, so duplicated
regular-file descriptors do *not* share a file offset (POSIX requires shared
open-file descriptions).  This matches the upstream Zephyr fdtable design and
is not exercised by this sample.

Building and Running
********************

The sample prints ``PASS``/``FAIL`` lines to the console and exits non-zero on
failure.

.. zephyr-app-commands::
   :zephyr-app: samples/posix/dup
   :board: qemu_x86
   :goals: run
   :compact:
