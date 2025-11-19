#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driverlib.h"
#include "board.h"

#include "diagport.h"

#define VERSION_INFO "XFS Beacon v0.0.1\0"

// diagnostic port commands
typedef enum 
{
    DIAGPORT_CMD_NONE,
    DIAGPORT_CMD_HELP,
    DIAGPORT_CMD_VERSION,
    DIAGPORT_CMD_STATUS,
    DIAGPORT_CMD_TOGGLE_HEALTH_LED,
    DIAGPORT_CMD_TOGGLE_SIDE_BLINK,
    DIAGPORT_CMD_TOGGLE_SONALERT,
} 
DIAGPORT_Command;

static char       cmd_buffer[256] = {0};
static uint16_t   cmd_index = 0;  

static DIAGPORT_Command parse_command_buffer(char *);
static void reset_command_buffer(void);

static void process_help_command(void);
static void process_version_command(void);
static void process_status_command(void);
static void process_toggle_health_led_command(void);
static void process_toggle_side_blink_command(void);
static void process_toggle_sonalert_command(void);

void DIAGPORT_initialize( void ) 
{
    reset_command_buffer();
}

void DIAGPORT_service( void )
{
    while(SCI_getRxFIFOStatus(SCIA_BASE) != 0) 
    {
        char ch = SCI_readCharNonBlocking(SCIA_BASE);
        
        if (ch == '\r')
        {
            DIAGPORT_Command command = parse_command_buffer(cmd_buffer);
        
            switch(command)
            {
                case DIAGPORT_CMD_HELP:
                {
                    process_help_command();
                    break;
                }

                case DIAGPORT_CMD_VERSION:
                {
                    process_version_command();
                    break;
                }

                case DIAGPORT_CMD_STATUS:
                {
                    process_status_command();
                    break;
                }

                case DIAGPORT_CMD_TOGGLE_HEALTH_LED:
                {
                    process_toggle_health_led_command();
                    break;
                }

                case DIAGPORT_CMD_TOGGLE_SIDE_BLINK:
                {
                    process_toggle_side_blink_command();
                    break;
                }

                case DIAGPORT_CMD_TOGGLE_SONALERT:
                {
                    process_toggle_sonalert_command();
                    break;
                }

                case DIAGPORT_CMD_NONE:
                default:
                {
                    DIAGPORT_transmit_string("\r\nUnknown Command\r\n>> \0");
                    break;
                }
            }
            reset_command_buffer();
        }

        else if (cmd_index >= sizeof(cmd_buffer)-1)
        {
            reset_command_buffer();
        }

        else
        {
            cmd_buffer[cmd_index++] = ch;
            cmd_buffer[cmd_index]   = '\0'; // null terminate
        }
    }
}

DIAGPORT_Command parse_command_buffer(char *buffer)
{
    if ((buffer[0] == '?') && (buffer[1] == '\0')) 
    {
        return DIAGPORT_CMD_HELP;
    }
    else if ((buffer[0] == 'v') && (buffer[1] == '\0')) 
    {
        return DIAGPORT_CMD_VERSION;
    }
    else if ((buffer[0] == 's') && (buffer[1] == '\0')) 
    {
        return DIAGPORT_CMD_STATUS;
    }
    else if ((buffer[0] == 'h') && (buffer[1] == '\0')) 
    {
        return DIAGPORT_CMD_TOGGLE_HEALTH_LED;
    }
    else if ((buffer[0] == 'b') && (buffer[1] == '\0')) 
    {
        return DIAGPORT_CMD_TOGGLE_SIDE_BLINK;
    }
    else if ((buffer[0] == 'a') && (buffer[1] == '\0')) 
    {
        return DIAGPORT_CMD_TOGGLE_SONALERT;
    }
    else if (buffer[0] != '\0')
    {
        return DIAGPORT_CMD_NONE;
    }

    return DIAGPORT_CMD_NONE;
}

void DIAGPORT_transmit_string(char *str)
{
    while(*str)
    {
        SCI_writeCharBlockingNonFIFO(DEBUG_SCI_BASE, *str++);
    }
}

void process_help_command(void)
{
    DIAGPORT_transmit_string("\r\n\0");
    DIAGPORT_transmit_string("Get Command Help:  ?\r\n\0");
    DIAGPORT_transmit_string("Get Version Info:  v\r\n\0");
    DIAGPORT_transmit_string("Get Status Info:   s\r\n\0");
    DIAGPORT_transmit_string("Toggle Health LED: h\r\n\0");
    DIAGPORT_transmit_string("Toggle Sonalert:   a\r\n\0");
    DIAGPORT_transmit_string("Toggle Side Blink: b\r\n\0");
    DIAGPORT_transmit_string(">> \0");
}

void process_version_command(void)
{
    DIAGPORT_transmit_string("\r\n\0");
    DIAGPORT_transmit_string(VERSION_INFO);
    DIAGPORT_transmit_string("\r\n>> \0");
}

void process_status_command(void)
{
    DIAGPORT_transmit_string("\r\n\0");
    // TODO: add status info here
    DIAGPORT_transmit_string(">> \0");
}

void process_toggle_health_led_command(void)
{
    static bool led_enabled = true;
    led_enabled = !led_enabled;

    MAIN_enable_health_led(led_enabled);
    
    DIAGPORT_transmit_string("\r\n\0");
    DIAGPORT_transmit_string(">> \0");
}

void process_toggle_side_blink_command(void)
{
    GPIO_togglePin(SIDE_BLINK);
    DIAGPORT_transmit_string("\r\n\0");
    DIAGPORT_transmit_string(">> \0");
}

void process_toggle_sonalert_command(void)
{
    GPIO_togglePin(SONALERT);
    DIAGPORT_transmit_string("\r\n\0");
    DIAGPORT_transmit_string(">> \0");
}

void reset_command_buffer(void)
{
    memset(cmd_buffer, 0, sizeof(cmd_buffer));
    cmd_index = 0;  
}

