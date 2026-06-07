SET(SOURCES ui_uiGraphPanel.c
    ui_Screen2.c
    ui_Screen3.c
    ui_Screen4.c
    ui_Screen5.c
    ui_Screen6.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_img_c925sls_png.c
    fonts/ui_font_Font1.c
    fonts/ui_font_Font12.c
    fonts/ui_font_Font13.c)

add_library(ui ${SOURCES})
