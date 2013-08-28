#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <xdo.h>
#include <mapper/mapper.h>

#define CLICK_SPEED 5
#define CLICK_LENGTH 5

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
            if (st->leave != NULL)
                st->leave((void**)&st, st->states[i], user_data);
            if (((State_t*)st->states[i])->enter != NULL)
                ((State_t*)st->states[i])->enter(&st->states[i], st, user_data);
            *state = st->states[i];
            return;
        }
    }
}

/*************/
void on_enter_wait(void** state, void* from, void* user_data)
{
    State_t* curr = *(State_t**)state;
    State_t* prev = (State_t*)from;

    static int frames = 0;

    if (strcmp(prev->name, "root") == 0)
        frames = 0;
    else if (strcmp(prev->name, "wait") == 0)
        frames++;
    else
        return;

    if (frames > CLICK_SPEED)
    {
        state_send_signal(&curr, "enough", NULL);
        *state = curr;
    }

    if (user_data == NULL)
        return;
    *(int*)user_data = frames;
}

/*************/
void on_enter_click(void** state, void* from, void* user_data)
{
    State_t* curr = *(State_t**)state;
    State_t* prev = (State_t*)from;

    static int frames = 0;

    if (strcmp(prev->name, "wait") == 0)
        frames = 0;
    else if (strcmp(prev->name, "click") == 0)
        frames++;
    else
        return;

    if (frames > CLICK_LENGTH)
    {
        state_send_signal(&curr, "clicked", NULL);
        *state = curr;
    }

    if (user_data == NULL)
        return;
    *(int*)user_data = frames;
}

/*************/
State_t* init_state_machine()
{
    State_t* root = state_new("root");
    State_t* wait = state_new("wait");
    State_t* click = state_new("click");
    State_t* move = state_new("move");

    state_set_enter_handler(wait, on_enter_wait);
    state_set_enter_handler(click, on_enter_click);

    state_add_signal(root, "position", wait);
    state_add_signal(wait, "position", wait);
    state_add_signal(wait, "enough", move);
    state_add_signal(wait, "no_position", click);
    state_add_signal(click, "clicked", root);
    state_add_signal(move, "no_position", root);

    return root;
}

/*************/
typedef struct
{
    xdo_t* display;
    float x, y;
    mapper_timetag_t timetag;
    int updated;
} Data_t;

/*************/
void depthmsg_handler(mapper_signal msig, mapper_db_signal props, int instance_id, void *value, int count, mapper_timetag_t* tt)
{
    Data_t* data = (Data_t*)props->user_data;
    xdo_t* xdo_display = data->display;

    int x, y;
    x = ((float*)value)[1];
    y = ((float*)value)[2];

    data->x = x;
    data->y = y;
    data->timetag = *tt;
    data->updated = 1;
}

/*************/
int main(int argc, char** argv)
{
    State_t* current = init_state_machine();

    xdo_t* xdo_display = xdo_new(NULL);

    Data_t data;
    data.display = xdo_display;
    data.x = 0.f;
    data.y = 0.f;
    data.updated = 0;

    mapper_device receiver = mdev_new("oscToMouse", 9700, 0);
    mapper_signal signal = mdev_add_input(receiver, "/blobserver", 4, 'f', 0, 0, 0, depthmsg_handler, &data);

    while(1)
    {
        mdev_poll(receiver, 50);
        int value;
        if (data.updated)
        {
            state_send_signal(&current, "position", &value);
        }
        else
        {
            state_send_signal(&current, "no_position", &value);
        }

        printf("%f %f - %s\n", data.x, data.y, current->name);

        if (strcmp(current->name, "move") == 0)
            xdo_mousemove(data.display, data.x, data.y, 0);
        else if (strcmp(current->name, "click") == 0)
            xdo_click(data.display, CURRENTWINDOW, 1);

        data.updated = 0;
    }
}
