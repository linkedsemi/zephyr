.. zephyr:code-sample:: helloworld_user
   :name: user mode test

Sample Output
=============

.. code-block:: console
*** Booting Zephyr OS build before_footprint-9-g6a225e996986 ***
   +user_function1---- 0x11111111
   -user_function2---- 0x22222222
   +user_function1---- 0x11111111
   -user_function2---- 0x22222222
   +user_function1---- 0x11111111
   -user_function2---- 0x22222222
   +user_function1---- 0x11111111
   -user_function2---- 0x22222222
   +user_function1---- 0x11111111
   -user_function2---- 0x22222222
   +user_function1---- 0x11111111
   -user_function2---- 0x22222222
   +user_function1---- 0x11111111
   -user_function2---- 0x22222222
   +user_function1---- 0x11111111
   -user_function2---- 0x22222222
   +user_function1---- 0x11111111
   -user_function2---- 0x22222222
   +user_function1---- 0x11111111
   -user_function2---- 0x22222222
   +user_function1---- 0x11111111
   -user_function2---- 0x22222222
   +user_function1---- 0x11111111
   +thread user_function1 can be abort !!!!!!!!!!
   -user_function2---- 0x22222222
   E:  mcause: 7, Store/AMO access fault
   E:   mtval: 0
   E:      a0: 00000000    t0: 0000005d
   E:      a1: 00000000    t1: 08005d60
   E:      a2: 08011098    t2: 00000000
   E:      a3: 00000001    t3: 00000030
   E:      a4: 00000366    t4: 00000020
   E:      a5: 00000034    t5: 00000023
   E:      a6: 08000000    t6: 08002690
   E:      a7: 00000000
   E:      sp: 10005eb0
   E:      ra: 080026f2
   E:    mepc: 080026f2
   E: mstatus: 00002080
   E:
   E: call trace:
   E:
   E: >>> ZEPHYR FATAL ERROR 0: CPU exception on CPU 0
   E: Current thread: 0x10000848 (unknown)
   -thread user_function1 is aborted !!!!!!!!!!
   -user_function2---- 0x22222222
   -user_function2---- 0x22222222
   -user_function2---- 0x22222222
   -user_function2---- 0x22222222
   -user_function2---- 0x22222222
   -user_function2---- 0x22222222
   -user_function2---- 0x22222222
   -user_function2---- 0x22222222
   -user_function2---- 0x22222222
   -user_function2---- 0x22222222
   -user_function2---- 0x22222222
   -user_function2---- 0x22222222
   -user_function2---- 0x22222222
   -user_function2---- 0x22222222
   -user_function2---- 0x22222222
