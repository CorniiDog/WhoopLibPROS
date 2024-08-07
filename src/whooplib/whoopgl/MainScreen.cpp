/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       MainScreen.cpp                                            */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 25 2024                                          */
/*    Description:  WhoopLib LVGL Main Screen                                 */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/whoopgl/MainScreen.hpp"
#include "whooplib/include/toolbox.hpp"
#include <string>
#include <sstream>

namespace whoop{

namespace screen{

// Animation callback function to scale the image
static void anim_scale_cb(void * var, int32_t v) {
    lv_obj_t * obj = (lv_obj_t *)var;
    lv_img_set_zoom(obj, (uint16_t)v);
}

static void anim_move_cb(void * var, int32_t v) {
    lv_obj_t * obj = (lv_obj_t *)var;

    lv_obj_set_pos(obj, -v*1.9, -v); // Move the object vertically
}

static void set_background_color(void * obj, int32_t t) {
    lv_obj_t * scr = static_cast<lv_obj_t*>(obj); // Get the current screen

    // Set the gradient colors and direction directly
    lv_obj_set_style_bg_color(scr, lv_color_make(t, 0, 0), LV_PART_MAIN);
    lv_obj_set_style_bg_grad_color(scr, lv_color_make(0, 0, t), LV_PART_MAIN);
    lv_obj_set_style_bg_grad_dir(scr, LV_GRAD_DIR_HOR, LV_PART_MAIN);
}

void draw_background_looped(){
    lv_obj_t * scr = lv_disp_get_scr_act(NULL); // Get the current screen

    // Create an animation for the gradient color change
    lv_anim_t anim;
    lv_anim_init(&anim);
    lv_anim_set_var(&anim, scr);
    lv_anim_set_exec_cb(&anim, set_background_color);
    lv_anim_set_time(&anim, 20000); // Animation time for one cycle (1000 ms)
    lv_anim_set_values(&anim, 0, 128); // Animation values from 0 to 128
    lv_anim_set_playback_time(&anim, 20000); // Time for reverse animation (1000 ms)
    lv_anim_set_repeat_count(&anim, LV_ANIM_REPEAT_INFINITE); // Repeat the animation indefinitely
    lv_anim_start(&anim);
}

static void anim_log_move_cb(void * var, int32_t v) {
    lv_obj_t * obj = (lv_obj_t *)var;

    lv_obj_set_pos(obj, v, 5); // Move the object vertically
}


void splash_screen(){
    lv_obj_t * scr = lv_disp_get_scr_act(NULL); // Get the current screen

    // Get the screen width and height
    lv_coord_t screen_width = lv_obj_get_width(scr);
    lv_coord_t screen_height = lv_obj_get_height(scr);

    // Create a new image object
    lv_obj_t * img = lv_img_create(scr);
    lv_img_set_src(img, &WhoopLibTiny); // Set the image source
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0); // Align to center of the parent


    // Ensure the image is set to its natural size initially
    lv_img_set_zoom(img, 256); // 256% to make it larger initially

    // Initialize and configure the animation
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, img);
    lv_anim_set_exec_cb(&a, anim_scale_cb);
    lv_anim_set_time(&a, 500); // Duration of the animation (1 second)
    lv_anim_set_values(&a, 600, 300); // Scale from large to normal size (256% to 100%)
    lv_anim_start(&a);

    pros::delay(2000); // 2 second delay

    // Move to top left corner of screen
    // Initialize and configure the animation
    lv_anim_set_exec_cb(&a, anim_move_cb);
    lv_anim_set_time(&a, 500); // Duration of the animation (1 second)
    lv_anim_set_values(&a, 0, 75); // Scale from large to normal size (256% to 100%)
    lv_anim_start(&a);

    // Initialize and configure the log animation
    lv_anim_t l_a;
    lv_anim_init(&l_a);
    lv_anim_set_var(&l_a, win);
    lv_anim_set_exec_cb(&l_a, anim_log_move_cb);
    lv_anim_set_time(&l_a, 500); // Duration of the animation (1 second)
    lv_anim_set_values(&l_a, 700, 205); // Scale from large to normal size (256% to 100%)
    lv_anim_start(&l_a);

}


void create_log_window(){
    lv_obj_t * scr = lv_disp_get_scr_act(NULL); // Get the current screen

    /*Create a window*/
    win = lv_win_create(scr, 25);
    lv_win_add_title(win, "Logs");                        /*Set the title*/
    lv_obj_set_size(win, 270, 230);
    lv_obj_set_pos(win, 700, 5);


    /* Add some dummy content to the window's content area */
    lv_obj_t * content = lv_win_get_content(win); // Get the window's content area
    txt = lv_label_create(content); // Create the label in the content area
    lv_label_set_text(txt, "");
}

void initialize(){
    if(screen_initialized)
        return;
    create_log_window();
    pros::Task backgroundTask(draw_background_looped, "Clean");
    pros::Task splashScreenTask(splash_screen, "Splash");

    screen_initialized = true;
}

} // screen

} // whoop