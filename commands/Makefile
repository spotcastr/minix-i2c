# Makefile for commands.

.include <bsd.own.mk>

SUBDIR=	add_route arp ash at backup btrace \
	cawf cd cdprobe cpp \
	chmod chown ci cleantmp cmp co \
	comm compress cp crc cron crontab cut \
	dd decomp16 DESCRIBE devmand devsize df dhcpd \
	dhrystone diff diskctl dumpcore \
	eject env factor fbdctl \
	find fix fold format fortune fsck.mfs \
	gcore gcov-pull getty grep hexdump host \
	hostaddr id ifconfig ifdef \
	intr ipcrm ipcs irdpd isoread last \
	less loadkeys loadramdisk logger look lp \
	lpd lspci mail MAKEDEV \
	mesg mined mkfifo \
	mount mt netconf nice nohup \
	nonamed od paste patch \
	ping postinstall poweroff pr prep printroot \
	profile progressbar pr_routes ps pwdauth \
	ramdisk rarpd rawspeed rcp readclock \
	reboot remsync rget rlogin \
	rotate rsh rshd service setup shar \
	sleep slip spell split sprofalyze sprofdiff srccrc \
	stty svclog svrctl swifi synctree sysenv \
	syslogd tail tcpd tcpdp tcpstat telnet \
	telnetd term termcap tget time touch \
	truncate tty udpstat umount uname \
	unstack update uud uue version vol \
	whereis which write writeisofs fetch \
	zdump zmodem pkgin_cd pkgin_all \
	worldstone updateboot update_bootcfg \
	atnormalize dosread fdisk loadfont \
	autopart part partition playwave  \
	recwave repartition screendump

.if ${MACHINE_ARCH} == "earm"
SUBDIR+=	eepromread
.endif

.include <bsd.subdir.mk>
