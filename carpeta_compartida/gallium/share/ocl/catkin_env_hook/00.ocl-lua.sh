#!/bin/sh

if [ -d "/opt/pal/gallium/share/lua/5.1" ]; then
    if [ "x$LUA_PATH" = "x" ]; then
        LUA_PATH=";"
    fi
    export LUA_PATH="$LUA_PATH;/opt/pal/gallium/share/lua/5.1/?.lua"
fi

if [ -d "/opt/pal/gallium/lib/lua/5.1" ]; then
    if [ "x$LUA_CPATH" = "x" ]; then
        LUA_CPATH=";"
    fi
    export LUA_CPATH="$LUA_CPATH;/opt/pal/gallium/lib/lua/5.1/?.so"
fi
