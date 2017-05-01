config watchdog
---

This package is part of motion meta package, it

	1. watch the directory containing motion config files
	2. publish msg when file are changed, and reload configs.

depend:
	1. watchdog
	2. rosparam load
