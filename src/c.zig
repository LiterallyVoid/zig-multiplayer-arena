pub usingnamespace @cImport({
    @cInclude("glad/gl.h");
    @cInclude("GLFW/glfw3.h");

    @cDefine("STB_TRUETYPE_IMPLEMENTATION", {});
    @cInclude("stb_truetype.h");
});
