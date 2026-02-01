#include "crane.h"

#include <string.h>

#include <esp_log.h>

#include <lownet.h>
#include <utility.h>
#include <serial_io.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#define CRANE_PROTO 0x05

#define TAG "crane"

void crane_connect(uint8_t id);
void crane_disconnect();
int  crane_action(uint8_t action); // returns zero if ACK is received
void crane_test(uint8_t id);
void crane_receive(const lownet_frame_t* frame);
void crane_send(uint8_t destination, const crane_packet_t* packet);

// state of a single flow
static struct
{
	uint16_t seq;
	uint8_t crane;
	QueueHandle_t acks;
	enum
		{
			ST_DISCONNECTED,
			ST_HANDSHAKE,
			ST_CONNECTED,
		} state;
} state;
// Track latest STATUS information
static uint16_t g_last_status_seq = 0;
static uint8_t  g_last_backlog    = 0;

int crane_init(void)
{
	if (lownet_register_protocol(CRANE_PROTO, crane_receive) != 0)
		{
			ESP_LOGE(TAG, "Failed to register crane protocol");
			return 1;
		}

	state.seq = 0;
	state.crane = 0;
	state.state = ST_DISCONNECTED;
	state.acks = xQueueCreate(8, sizeof(uint16_t));
	// status tracking
        g_last_status_seq = 0;
        g_last_backlog    = 0;
	return 0;
}

void crane_command(char* args)
{
	if (!args)
		{
			serial_write_line("Missing argument COMMAND");
			return;
		}
	char* saveptr;

	char* command = strtok_r(args, " ", &saveptr);
	if (!command)
		{
			serial_write_line("Missing argument COMMAND");
			return;
		}

	if (strcmp(command, "help") == 0)
		{
			serial_write_line("open ID    Connect to a crane at ID");
			serial_write_line("close      Close an existing connection");
			serial_write_line("test ID    Connect to ID in test mode and execute test pattern");
			serial_write_line("CMD        Implementation defined commands to trigger crane actions");
		}
	else if (strcmp(command, "open") == 0)
		{
			char* id = strtok_r(NULL, " ", &saveptr);
			if (!id)
				{
					serial_write_line("Missing argument ID");
					return;
				}
			uint8_t dest = hex_to_dec(id + 2);
			crane_connect(dest);
		}
	else if (strcmp(command, "close") == 0)
		{
			crane_disconnect();
		}
	else if (strcmp(command, "test") == 0)
		{
			char* id = strtok_r(NULL, " ", &saveptr);
			if (!id)
				{
					serial_write_line("Missing argument ID");
					return;
				}
			uint8_t dest = hex_to_dec(id + 2);
			crane_test(dest);
		}
	else
		{
			uint8_t action;
			switch (command[0])
                        {
                        case 'f':  // forward
	                        action = CRANE_FWD;
	                        break;
                        case 'b':  // backward
	                        action = CRANE_REV;
	                        break;
                        case 'u':  // up
	                        action = CRANE_UP;
	                        break;
                        case 'd':  // down
	                        action = CRANE_DOWN;
	                        break;
                        case 'o':  // light on
	                        action = CRANE_LIGHT_ON;
	                        break;
                        case 'O':  // capital O -> light off
	                        action = CRANE_LIGHT_OFF;
	                        break;
                        case 's':  // stop
	                        action = CRANE_STOP;
	                        break;
                        default:
	                        ESP_LOGI(TAG, "Invalid crane command");
	                        return;
                        }
                        crane_action(action);

		}
}

void crane_recv_connect(const crane_packet_t* packet)
{
    ESP_LOGI(TAG, "Received CONNECT packet");

    if (state.state != ST_HANDSHAKE)
        return;

    ESP_LOGI(TAG, "packet flags: %02x", packet->flags);

    // Expect SYN|ACK
    if ((packet->flags & (CRANE_SYN | CRANE_ACK)) != (CRANE_SYN | CRANE_ACK))
    {
        ESP_LOGW(TAG, "Invalid handshake flags, expected SYN|ACK");
        return;
    }

        // Prepare final ACK packet
    crane_packet_t outpkt;
    memset(&outpkt, 0, sizeof(outpkt));
    outpkt.type  = CRANE_CONNECT;
    outpkt.seq   = 0;     // handshake always uses seq = 0

    // Always ACK — and if crane used TEST, we must keep TEST in final ACK
    outpkt.flags = CRANE_ACK;
    if (packet->flags & CRANE_TEST) {
        outpkt.flags |= CRANE_TEST;
    }

    outpkt.d.conn.challenge = ~packet->d.conn.challenge;

    crane_send(state.crane, &outpkt);

    // After successful handshake, first ACTION must use seq = 1
    state.seq = 1;
    state.state = ST_CONNECTED;


    ESP_LOGI(TAG, "Connection established with crane 0x%02x", state.crane);
}


void crane_recv_close(const crane_packet_t* packet)
{
	ESP_LOGI(TAG, "Closing connection");
	state.seq = 0;
	state.state = ST_DISCONNECTED;
	state.crane = 0;
}

void crane_recv_status(const crane_packet_t* packet)
{
    char buffer[200];

    if (packet->flags & CRANE_NAK)
    {
        ESP_LOGI(TAG, "Received status packet with NAK -- not in use yet");
        return;
    }
    else
    {
        // Only treat as ACK if seq looks valid (non-zero, not 0xFFFF)
        if (packet->seq != 0 && packet->seq != 0xFFFF) {
            xQueueSend(state.acks, &packet->seq, 0);
        }
    }

    // --- NEW: track last status info ---
    g_last_status_seq = packet->seq;
    g_last_backlog    = packet->d.status.backlog;
    // -----------------------------------

    snprintf(buffer, sizeof buffer,
             "backlog: %d\n"
             "time: %d\n"
             "light: %s\n",
             packet->d.status.backlog,
             packet->d.status.time_left,
             packet->d.status.light ? "on" : "off");
    serial_write_line(buffer);
}


void crane_receive(const lownet_frame_t* frame)
{
	crane_packet_t packet;
	memcpy(&packet, frame->payload, sizeof packet);
	ESP_LOGI(TAG, "Received packet frame from %02x, type: %d", frame->source, packet.type);
	switch (packet.type)
		{
		case CRANE_CONNECT:
			crane_recv_connect(&packet);
			break;
		case CRANE_STATUS:
			crane_recv_status(&packet);
			break;
		case CRANE_ACTION:
			break;
		case CRANE_CLOSE:
			crane_recv_close(&packet);
		}
}

/*
 * This function starts the connection establishment
 * procedure by sending a SYN packet to the given node.
 */
void crane_connect(uint8_t id)
{
    if (state.state != ST_DISCONNECTED)
        return;

    crane_packet_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.type = CRANE_CONNECT;
    packet.flags = CRANE_SYN;
    packet.seq = 0;                 // handshake always uses seq = 0
    packet.d.conn.challenge = 0;    // initial challenge = 0

    state.crane = id;
    state.state = ST_HANDSHAKE;
    state.seq = 0;                  // reset sequence counter for new connection

    crane_send(id, &packet);
}

void crane_disconnect(void)
{
	crane_packet_t packet;

	// ------------------------------------------------
	// Milestone I, Task 3: construct a CLOSE packet and
	// send it to the (right) crane!
	{
		memset(&packet, 0, sizeof(packet));
                packet.type  = CRANE_CLOSE;
                packet.flags = 0;          // no flags
                packet.seq   = state.seq;  // next sequence
                packet.d.close = 0;        // reserved must be zero

                crane_send(state.crane, &packet);
                ESP_LOGI(TAG, "Sent CLOSE packet to crane 0x%02x", state.crane);

                // wait a bit for possible ACK before resetting
                vTaskDelay(pdMS_TO_TICKS(500));  
                state.state = ST_DISCONNECTED;
                state.seq = 0;
	}
	// ------------------------------------------------
	// Then we update our state
	state.state = ST_DISCONNECTED;
	state.seq = 0;
}


/*
 *	Subroutine for crane_action: read ACKs from crane, blocks for some time
 */
uint16_t read_acks(void)
{
	uint16_t seq, x;

	// Wait for an ack up to 5 seconds
	if ( xQueueReceive(state.acks, &seq, 5000/portTICK_PERIOD_MS) != pdTRUE )
		seq = 0;
	// read any other acks if in the queue
	while ( xQueueReceive(state.acks, &x, 0) == pdTRUE )
		seq = seq >= x ? seq : x;
	return seq;
}

/*
 *	This can block for a while if no immediate ACK
 */
int crane_action(uint8_t action)
{
    if (state.state != ST_CONNECTED) {
        ESP_LOGW(TAG, "Cannot send action, not connected");
        return -1;
    }

    if (state.crane == 0x00) {
        ESP_LOGE(TAG, "state.crane is 0x00 — refusing to send");
        state.state = ST_DISCONNECTED;
        return -1;
    }

    crane_packet_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.type = CRANE_ACTION;
    packet.seq  = state.seq;               // use current seq
    packet.d.action.cmd = action;
    memset(packet.d.action.reserved, 0, sizeof(packet.d.action.reserved));
    
    ESP_LOGI(TAG, "Sending ACTION cmd=%d seq=%d", action, packet.seq);

    crane_send(state.crane, &packet);

    for (int attempt = 0; attempt < 5; ++attempt)
    {
        uint16_t ack = read_acks();  // wait up to 5s

        if (ack == 0) {
            ESP_LOGW(TAG, "No ACK, retransmitting (try %d)", attempt + 1);
            crane_send(state.crane, &packet);
            continue;
        }

        if (ack == 0xFFFF) {
            ESP_LOGW(TAG, "Spurious ACK seq 0xFFFF, ignoring");
            continue;   // don't retransmit just because of this
        }

        if (ack > state.seq) {
            ESP_LOGE(TAG, "Unexpected ACK seq (%d > %d)", ack, state.seq);
            crane_disconnect();
            return -2;
        }

        if (ack == state.seq) {
            state.seq++;   // next command will use next seq
            ESP_LOGI(TAG, "ACK received for seq %d", ack);
            return 0;
        }
    }

    ESP_LOGI(TAG, "Received no ack from node=0x%02x", state.crane);
    crane_disconnect();
    return -1;
}




// ------------------------------------------------
//
// Milestone III: run the test pattern
//
// 1. establish connection with TEST flag
// 2. run the test pattern according to the specs
// 3. close the connection
//
// Hint: you can work it all out here slowly, or be a wizard
//       and launch a separate task for this!
//

static void crane_wait_until_idle(void)
{
    // Wait up to ~10 seconds for backlog to become 0
    for (int i = 0; i < 100; ++i)  // 100 * 100 ms = 10 seconds
    {
        if (g_last_backlog == 0) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGW(TAG, "Timeout waiting for backlog to drain (backlog=%d)", g_last_backlog);
}


void crane_test(uint8_t id)
{
    ESP_LOGI(TAG, "Starting automated crane test with 0x%02x", id);

    // Start from a clean state
    state.crane = id;
    state.state = ST_DISCONNECTED;
    state.seq   = 0;

    // 1. Connect with TEST flag
    crane_packet_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.type  = CRANE_CONNECT;
    packet.flags = CRANE_SYN | CRANE_TEST;
    packet.seq   = 0;                 // handshake seq = 0
    packet.d.conn.challenge = 0;

    state.crane = id;
    state.state = ST_HANDSHAKE;
    state.seq   = 0;                  // handshake uses seq 0

    crane_send(id, &packet);

    // Wait up to ~3 seconds for handshake to complete
    for (int i = 0; i < 30 && state.state != ST_CONNECTED; ++i) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (state.state != ST_CONNECTED)
    {
        ESP_LOGW(TAG, "Handshake failed");
        return;
    }

    // At this point crane_recv_connect() has set state.seq = 1

    // 2. Run the test sequence

    // 2. Switch on the light
    crane_action(CRANE_LIGHT_ON);

    // 3. Drive crane forward for two steps
    crane_action(CRANE_FWD);
    crane_action(CRANE_FWD);

    // 4. Drive crane backward for one step
    crane_action(CRANE_REV);

    // 5. Pause until all commands have executed
    crane_wait_until_idle();

    // 6. Lower the hook for two steps
    crane_action(CRANE_DOWN);
    crane_action(CRANE_DOWN);

    // 7. Pause until all commands have executed
    crane_wait_until_idle();

    // 8. Lift the hook back up for two steps
    crane_action(CRANE_UP);
    crane_action(CRANE_UP);

    // 9. Drive crane backwards for one step
    crane_action(CRANE_REV);

    // 10. Switch off the light
    crane_action(CRANE_LIGHT_OFF);

    // Optionally wait for everything to be flushed
    crane_wait_until_idle();

    // 11. Disconnect
    crane_disconnect();

    ESP_LOGI(TAG, "Test sequence completed");
}



void crane_send(uint8_t id, const crane_packet_t* packet)
{
	lownet_frame_t frame;
	frame.destination = id;
	frame.protocol = CRANE_PROTO;
	frame.length = sizeof *packet;
	memcpy(frame.payload, packet, sizeof *packet);

	lownet_send(&frame);
}
