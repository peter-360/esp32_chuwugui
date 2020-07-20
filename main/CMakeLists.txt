# idf_component_register(SRCS "play_mp3_control_example.c"
#                             "as608.c"
#                     INCLUDE_DIRS ".")

set(COMPONENT_SRCS ./play_mp3_control_example.c
                    ./as608.c)
set(COMPONENT_ADD_INCLUDEDIRS "")

# set(COMPONENT_SRCS ./as608.c)
# set(COMPONENT_ADD_INCLUDEDIRS "")
set(COMPONENT_EMBED_TXTFILES adf_music.mp3)

register_component()