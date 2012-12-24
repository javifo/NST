PWDD=`pwd`

make omap3621_gossamer_evt1c_defconfig ARCH=arm CROSS_COMPILE=$PWDD/../arm-linux-androideabi-4.4.3/bin/arm-linux-androideabi-
make uImage ARCH=arm CROSS_COMPILE=$PWDD/../arm-linux-androideabi-4.4.3/bin/arm-linux-androideabi-
