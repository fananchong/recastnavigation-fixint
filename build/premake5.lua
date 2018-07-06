workspace "recastnavigation-fixint"
    configurations { "Debug", "Release" }
    platforms { "x32", "x64" }
    targetdir "../bin/"
    language "C++"
    includedirs {
        "..",
        "../Contrib",
        "../Contrib/fastlz",
        "../Contrib/libfixmath",
        "../Contrib/SDL/include",
        "../DebugUtils/Include",
        "../Detour/Include",
        "../DetourCrowd/Include",
        "../DetourTileCache/Include",
        "../Recast/Include/",
        "../RecastDemo/Include/",
    }
    flags {
        "C++11",
        "StaticRuntime",
    }

    filter "configurations:Debug"
    defines { "_DEBUG" }
    flags { "Symbols" }
    libdirs { }
    filter "configurations:Release"
    defines { "NDEBUG" }
    libdirs { }
    optimize "On"
    filter { }
    

if os.is("windows") then
project "recast"
    kind "ConsoleApp"
    targetname "recast"
    libdirs { "../bin" }
    files {
        "../Contrib/**",
        "../DebugUtils/**",
        "../Detour/**",
        "../DetourCrowd/**",
        "../DetourTileCache/**",
        "../Recast/**",
        "../RecastDemo/**",
    }
    defines { "WIN32", "_WINDOWS", "_CRT_SECURE_NO_WARNINGS", "_HAS_EXCEPTIONS=0" }
    
    configuration { "windows" }
	links { 
		"glu32",
		"opengl32",
		"SDL2",
		"SDL2main",
	}
    filter { "platforms:x32" }
        libdirs { "../Contrib/SDL/lib/x86" }
        postbuildcommands {
            -- Copy the SDL2 dll to the Bin folder.
            '{COPY} "%{wks.location}../Contrib/SDL/lib/x86/SDL2.dll" "%{cfg.targetdir}"'
        }
    filter { "platforms:x64" }
        libdirs { "../Contrib/SDL/lib/x64" }
        postbuildcommands {
            -- Copy the SDL2 dll to the Bin folder.
            '{COPY} "%{wks.location}../Contrib/SDL/lib/x64/SDL2.dll" "%{cfg.targetdir}"'
        }
end
