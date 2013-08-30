#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <opencv/cv.h>
#include <mapper/mapper.h>
#include <xdo.h>

#define CLICK_SPEED 5
#define CLICK_LENGTH 5
#define MARKER_SIZE 128

int gIsRunning;

/*************/
typedef void state_on_enter_handler(void** state, void* from, void* user_data);
typedef void state_on_leave_handler(void** state, void* to, void* user_data);

typedef struct
{
    char* name; 
    state_on_enter_handler* enter;
    state_on_leave_handler* leave;
    int nbr;
    char** signals;
    void** states;
} State_t;

State_t* state_new(char* name)
{
    State_t* state = (State_t*)malloc(sizeof(State_t));
    state->name = name;
    state->enter = NULL;
    state->leave = NULL;
    state->nbr = 0;
    state->signals = (char**)malloc(sizeof(char*));
    state->states = (void**)malloc(sizeof(State_t*));
}

State_t* state_free(State_t* state)
{
    free(state->signals);
    free(state->states);
    free(state);
}

void state_add_signal(State_t* state, char* signal, State_t* output_state)
{
    if (state == NULL || signal == NULL || output_state == NULL)
        return;

    state->signals = (char**)realloc(state->signals, (state->nbr + 2) * sizeof(char*));
    state->signals[state->nbr] = signal;

    state->states = (void**)realloc(state->states, (state->nbr + 2) * sizeof(void*));
    state->states[state->nbr] = (void*)output_state;

    state->nbr++;
}

void state_set_enter_handler(State_t* state, state_on_enter_handler* handler)
{
    if (state == NULL || handler == NULL)
        return;

    state->enter = handler;
}

void state_set_leave_handler(State_t* state, state_on_leave_handler* handler)
{
    if (state == NULL || handler == NULL)
        return;

    state->leave = handler;
}

void state_send_signal(State_t** state, char* signal, void* user_data)
{
    if (state == NULL || signal == NULL)
        return;

    State_t* st = *state;

    for (int i = 0; i < st->nbr; ++i)
    {
        if (strcmp(signal, st->signals[i]) == 0)
        {
            State_t* destination = st->states[i];

            if (st->leave != NULL)
                st->leave((void**)&st, destination, user_data);
            if (((State_t*)st->states[i])->enter != NULL)
                ((State_t*)st->states[i])->enter((void**)&destination, st, user_data);
            *state = destination;
            return;
        }
    }
}

/*************/
typedef struct
{
    xdo_t* display;
    float x, y;
    int contact;
    mapper_timetag_t timetag;
    int updated;
} Data_t;

typedef struct
{
    int set;
    float x, y;
} Calib_t;

/*************/
void on_enter_move(void** state, void* from, void* user_data)
{
    State_t* curr = *(State_t**)state;
    State_t* prev = (State_t*)from;
    Data_t* data = user_data;

    xdo_mousemove(data->display, data->x, data->y, 0);
}

/*************/
void on_enter_click(void** state, void* from, void* user_data)
{
    State_t* curr = *(State_t**)state;
    State_t* prev = (State_t*)from;
    Data_t* data = user_data;

    static int frames = 0;

    if (strcmp(prev->name, "move") == 0)
    {
        xdo_mousemove(data->display, data->x, data->y, 0);
        frames = 0;
    }
    else if (strcmp(prev->name, "click") == 0)
    {
        xdo_mousemove(data->display, data->x, data->y, 0);
        frames++;

        if (frames == CLICK_SPEED)
            xdo_mousedown(data->display, CURRENTWINDOW, 1);
    }
}

/*************/
void on_leave_click(void** state, void* to, void* user_data)
{
    State_t* curr = *(State_t**)state;
    State_t* next = (State_t*)to;
    Data_t* data = user_data;

    if (strcmp(next->name, "click") != 0)
        xdo_mouseup(data->display, CURRENTWINDOW, 1);
}

/*************/
State_t* init_state_machine()
{
    State_t* root = state_new("root");
    State_t* click = state_new("click");
    State_t* move = state_new("move");

    state_set_enter_handler(click, on_enter_click);
    state_set_leave_handler(click, on_leave_click);

    state_add_signal(root, "position", move);
    state_add_signal(move, "contact", click);
    state_add_signal(move, "no_position", root);
    state_add_signal(click, "position", move);
    state_add_signal(click, "contact", click);
    state_add_signal(click, "no_position", root);

    return root;
}

/*************/
void free_state_machine(State_t* root)
{
}

/*************/
void mousemsg_handler(mapper_signal msig, mapper_db_signal props, int instance_id, void *value, int count, mapper_timetag_t* tt)
{
    Data_t* data = (Data_t*)props->user_data;

    int x, y, contact;
    x = ((float*)value)[1];
    y = ((float*)value)[2];
    contact = ((float*)value)[5];

    data->x = x;
    data->y = y;
    data->contact = contact;
    data->timetag = *tt;
    data->updated = 1;
}

/*************/
void calibmsg_handler(mapper_signal msig, mapper_db_signal props, int instance_id, void* value, int count, mapper_timetag_t* tt)
{
    Calib_t* calib = (Calib_t*)props->user_data;

    int id, x, y;
    id = ((float*)value)[0];
    x = ((float*)value)[1];
    y = ((float*)value)[2];

    if (id < 4)
    {
        calib[id].set = 1;
        calib[id].x = x;
        calib[id].y = y;
    }
}

/*************/
void leave(int sig)
{
    gIsRunning = 0;
}

/*************/
int main(int argc, char** argv)
{
    (void) signal(SIGINT, leave);

    int msgLength = 4;
    if (argc > 1)
        msgLength = atoi(argv[1]);

    State_t* root = init_state_machine();
    State_t* current = root;

    xdo_t* xdo_display = xdo_new(NULL);

    Data_t data;
    data.display = xdo_display;
    data.x = 0.f;
    data.y = 0.f;
    data.contact = 0;
    data.updated = 0;

    Calib_t calib[4];
    for (int i = 0; i < 4; ++i)
        calib[i].set = 0;

    mapper_device receiver = mdev_new("oscToMouse", 9700, 0);
    mapper_signal sigMouse = mdev_add_input(receiver, "/mouse", 6, 'f', 0, 0, 0, mousemsg_handler, &data);
    mapper_signal sigCalib = mdev_add_input(receiver, "/calibration", 4, 'f', 0, 0, 0, calibmsg_handler, &calib);

    gIsRunning = 1;
    while(gIsRunning)
    {
        mdev_poll(receiver, 25);
        int value;

        int allSet = 1;
        for (int i = 0; i < 4; ++i)
        {
            if (calib[i].set)
                printf("%i - %f %f\n", i, calib[i].x, calib[i].y);
            else
                allSet = 0;
        }
        if (allSet)
        {
            CvPoint2D32f points[4];
            for (int i = 0; i < 4; ++i)
            {
                points[i].x = calib[i].x;
                points[i].y = calib[i].y;
            }

            CvPoint2D32f outPoints[4];
            outPoints[0].x = MARKER_SIZE;
            outPoints[0].y = MARKER_SIZE;
            outPoints[1].x = 1024.f - MARKER_SIZE;
            outPoints[1].y = MARKER_SIZE;
            outPoints[2].x = MARKER_SIZE;
            outPoints[2].y = 768.f - MARKER_SIZE;
            outPoints[3].x = 1024.f - MARKER_SIZE;
            outPoints[3].y = 768.f - MARKER_SIZE;

            CvMat* perspectiveMatrix = cvCreateMat(3, 3, CV_64F);
            cvGetPerspectiveTransform(points, outPoints, perspectiveMatrix);

            CvMat* p = cvCreateMat(1, 1, CV_64FC2);
            CvMat* h = cvCreateMat(1, 1, CV_64FC2);

            CvScalar current;
            current.val[0] = data.x;
            current.val[1] = data.y;

            cvSet2D(p, 0, 0, current);
            cvPerspectiveTransform(p, h, perspectiveMatrix);
            current = cvGet2D(h, 0, 0);

            if (current.val[0] > 0.f && current.val[0] < 1600.f)
                data.x = current.val[0];
            else
                data.x = 800.f;
            
            if (current.val[1] > 0.f && current.val[1] < 1200.f)
                data.y = current.val[1];
            else
                data.y = 600.f;

            printf("--- %f %f\n", current.val[0], current.val[1]);
        }

        if (data.contact)
            state_send_signal(&current, "contact", &data);
        else if (data.updated)
            state_send_signal(&current, "position", &data);
        else
            state_send_signal(&current, "no_position", &data);

        printf("%f %f - %i - %s\n", data.x, data.y, data.contact, current->name);

        if (data.updated)
            xdo_mousemove(data.display, data.x, data.y, 0);
        data.updated = 0;
    }

    free_state_machine(root);
}
