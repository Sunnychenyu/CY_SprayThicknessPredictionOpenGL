function(smrobot_generate_shader_resource_source OUT_VAR SHADER_ROOT)
    get_filename_component(_smrobot_shader_root "${SHADER_ROOT}" ABSOLUTE)
    if(NOT IS_DIRECTORY "${_smrobot_shader_root}")
        message(FATAL_ERROR "Shader resource root does not exist: ${_smrobot_shader_root}")
    endif()

    file(GLOB_RECURSE _smrobot_shader_files CONFIGURE_DEPENDS
        "${_smrobot_shader_root}/*.vert"
        "${_smrobot_shader_root}/*.frag"
        "${_smrobot_shader_root}/*.geom"
        "${_smrobot_shader_root}/*.glsl"
    )
    list(SORT _smrobot_shader_files)

    set(_smrobot_output_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
    file(MAKE_DIRECTORY "${_smrobot_output_dir}")
    set(_smrobot_output_file "${_smrobot_output_dir}/EmbeddedShaderSources.cpp")

    set(_smrobot_source "#include <RenderCore/EmbeddedShaderResourceApi.h>\n\n")
    string(APPEND _smrobot_source "#include <cstddef>\n\n")
    string(APPEND _smrobot_source "#if defined(_WIN32)\n")
    string(APPEND _smrobot_source "#define SMROBOT_SHADER_RESOURCE_EXPORT __declspec(dllexport)\n")
    string(APPEND _smrobot_source "#else\n")
    string(APPEND _smrobot_source "#define SMROBOT_SHADER_RESOURCE_EXPORT\n")
    string(APPEND _smrobot_source "#endif\n\n")
    string(APPEND _smrobot_source "namespace\n{\n")

    set(_smrobot_entries "")
    set(_smrobot_index 0)
    foreach(_smrobot_shader_file IN LISTS _smrobot_shader_files)
        file(RELATIVE_PATH _smrobot_logical_path "${_smrobot_shader_root}" "${_smrobot_shader_file}")
        string(REPLACE "\\" "/" _smrobot_logical_path "${_smrobot_logical_path}")
        file(READ "${_smrobot_shader_file}" _smrobot_shader_text)
        string(SHA256 _smrobot_shader_hash "${_smrobot_logical_path}\n${_smrobot_shader_text}")
        string(SUBSTRING "${_smrobot_shader_hash}" 0 16 _smrobot_shader_token)

        set(_smrobot_symbol "kShaderSource_${_smrobot_index}")
        string(APPEND _smrobot_source
            "    constexpr const char ${_smrobot_symbol}[] = R\"${_smrobot_shader_token}("
            "${_smrobot_shader_text}"
            ")${_smrobot_shader_token}\";\n\n"
        )
        string(APPEND _smrobot_entries
            "        { \"${_smrobot_logical_path}\", ${_smrobot_symbol}, sizeof(${_smrobot_symbol}) - 1 },\n"
        )
        math(EXPR _smrobot_index "${_smrobot_index} + 1")
    endforeach()

    string(APPEND _smrobot_source
        "    constexpr rendercore::ShaderResourceEntry kShaderResourceEntries[] = {\n"
        "${_smrobot_entries}"
        "    };\n"
        "}\n\n"
        "extern \"C\" SMROBOT_SHADER_RESOURCE_EXPORT\n"
        "const rendercore::ShaderResourceEntry* smrobotGetShaderResourceEntries(std::size_t* count)\n"
        "{\n"
        "    if (count)\n"
        "        *count = sizeof(kShaderResourceEntries) / sizeof(kShaderResourceEntries[0]);\n"
        "    return kShaderResourceEntries;\n"
        "}\n"
    )

    file(WRITE "${_smrobot_output_file}" "${_smrobot_source}")
    set(${OUT_VAR} "${_smrobot_output_file}" PARENT_SCOPE)
endfunction()
