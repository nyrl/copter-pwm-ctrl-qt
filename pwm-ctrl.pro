TEMPLATE =	app

CONFIG +=	debug_and_release \
		warn_on \
		copy_dir_files

debug:CONFIG +=	console

CONFIG -=	warn_off


TARGET = pwm-ctrl

SOURCE +=	main.cpp
HEADERS +=	main.h
FORMS +=	main_window.ui


unix {
	target.path =	$$[INSTALL_ROOT]/bin
	INSTALLS +=	target
}

