Sentio Framework
================

This file contains the kernel, hardware abstraction layer and libraries for
development on the Sentio platform.


Clone the project
-----------------

This code is intended to be included as a submodule in a Sentio application.
The code can be neither compiled nor executed on its own. Nevertheless, to
inspect the code or make changes it can be cloned.

    git clone git://github.com/mpfusion/sentio-framework.git


Install
-------

### CodeSourcery gcc tools

These tools can be obtained as binary version and source code from
[Mentor Graphics](http://www.mentor.com). It is also available in the [Downloads
section](https://github.com/mpfusion/sentio-framework/downloads).

Unpack the archive and add the `bin` directory to your `PATH`, e.g. as
follows:

    cat >> $HOME/.profile << EOL
    if [ -d "\$HOME/usr/opt/arm-2012.09/bin" ] ; then
        PATH="\$HOME/usr/opt/arm-2012.09/bin:\$PATH"
    fi
    EOL


### Energy Mirco energyAware tools

These tools are only available as binaries and can be obtained from
[energymicro](http://www.energymicro.com/). Here is a [link to the Linux 686
binary
package](http://cdn.energymicro.com/dl/packages/energyAwareTools_25052012.tgz)
If this download is unavailable it can be found in the [Downloads
section](https://github.com/mpfusion/sentio-framework/downloads).

Unpack the archive and add the `bin` directory to your `PATH`, e.g. as
follows:

    cat >> $HOME/.profile << EOL
    if [ -d "\$HOME/usr/opt/energymicro" ] ; then
        PATH="\$HOME/usr/opt/energymicro:\$PATH"
    fi
    EOL

For this tools you need the 32 bit `readline` and `libusb` libraries.
Depending on your distribution the packages might be called differently, on
Debian it is `lib32readline5` and `libusb-1.0-0`.


### Terminal access

To see the debug output one can use a terminal program like `minicom`. The
necessary settings are as follows.

    device         /dev/ttyUSB0
    baud rate      2000000
    bits           8
    parity         odd
    flow control   no

This can be accomplished with the following minicom configuration.

    pu port      /dev/ttyUSB0
    pu baudrate  2000000
    pu bits      8
    pu parity    O
    pu stopbits  1

Place these lines in a file called `$HOME/.minirc.dfl` and running `minicom`
on the console should display the debug output generated by the running
program. Unfortunately by default one needs root access for this.


### Code style

Tabs are used for indentation and spaces for alignment. To maintain a uniform
code appearance the program `astyle` is used with the following options.

	astyle -T4pbcUDH -z2
