#pragma once

extern int _overlay_on;
extern int overlay_on();
extern void overlay_cursor(int x, int y);
extern void dprint(const char *fmt, ...);
extern void overlay_printf(const char *fmt, ...);
extern void overlay_clear();
extern void overlay_status(const char *fmt, ...);
extern void overlay_message(const char *msg, int center);
extern void overlay(int state);     // turn overlay on/off
