# set(COMPONENT_SRCS ./play_mp3_control_example.c
#                     ./as608.c
#                     ../server_certs/ca_cert.pem)
# set(COMPONENT_ADD_INCLUDEDIRS "")

idf_component_register(SRCS "play_mp3_control_example.c"
                            "as608.c"
                            INCLUDE_DIRS "."
                            EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem)

# set(COMPONENT_EMBED_TXTFILES 16b-2c-8000hz.mp3 adf_music.mp3 16b-2c-22050hz.mp3 16b-2c-44100hz.mp3)

register_component()