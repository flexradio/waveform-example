// SPDX-License-Identifier: LGPL-3.0-or-later
/// @file main.c
/// @brief Functional test yoke for Waveform SDK Functionality
/// @authors Annaliese McDermond <anna@flex-radio.com>
///
/// @copyright Copyright (c) 2020 FlexRadio Systems
///
/// This program is free software: you can redistribute it and/or modify
/// it under the terms of the GNU Lesser General Public License as published by
/// the Free Software Foundation, version 3.
///
/// This program is distributed in the hope that it will be useful, but
/// WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
/// Lesser General Public License for more details.
///
/// You should have received a copy of the GNU Lesser General Public License
/// along with this program. If not, see <http://www.gnu.org/licenses/>.
///

// ****************************************
// System Includes
// ****************************************
#include <arpa/inet.h>
#include <getopt.h>
#include <libgen.h>
#include <netdb.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ****************************************
// Project Includes
// ****************************************
#include <waveform/waveform_api.h>

// ****************************************
// Structs, Enums, typedefs
// ****************************************

// A structure to hold context for the waveform.  This can be passed as a pointer to the callback registration functions
// so that they have access to waveform common data.  We keep things in here like the current phase of the sine wave
// for both the TX and RX sides of things.
struct junk_context {
    _Atomic uint8_t rx_phase;
    _Atomic uint8_t tx_phase;
    bool tx;
    int16_t snr;
    uint64_t byte_data_counter;
};

// ****************************************
// Macros
// ****************************************
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

// ****************************************
// Static Variables
// ****************************************

// The values for a 1000Hz sine wave at 24kHz sample rate precalculated to save processor time.
static const float sin_table[] = {
    0.0F,
    0.25881904510252074F,
    0.49999999999999994F,
    0.7071067811865475F,
    0.8660254037844386F,
    0.9659258262890682F,
    1.0F,
    0.9659258262890683F,
    0.8660254037844388F,
    0.7071067811865476F,
    0.5000000000000003F,
    0.258819045102521F,
    1.2246467991473532e-16F,
    -0.25881904510252035F,
    -0.4999999999999998F,
    -0.7071067811865471F,
    -0.8660254037844384F,
    -0.9659258262890681F,
    -1.0F,
    -0.9659258262890684F,
    -0.866025403784439F,
    -0.7071067811865477F,
    -0.5000000000000004F,
    -0.2588190451025215F
};

//  A set of meters that we intend to send to the radio.  Each meter has a name, minimum and maximum value, and a unit
//  associated with it.  See the documentation for all the different units supported.
static const struct waveform_meter_entry meters[] = {
    {.name = "junk-snr", .min = -100.0f, .max = 100.0f, .unit = DB},
    {.name = "junk-foff", .min = 0.0f, .max = 100000.0f, .unit = DB},
    {.name = "junk-clock-offset", .min = 0.0f, .max = 100000.0f, .unit = DB}
};


// ****************************************
// Static Functions
// ****************************************

/// \brief An example "status" callback
/// A callback that merely echos the arguments we receive.  This is used as the "status" callback in the main program
/// to receive any status updates we have subscribed to in the radio.
/// \param waveform A pointer to the opaque waveform structure returned by waveform_create
/// \param argc The number of arguments in the status command
/// \param argv An array of the arguments in the status command
/// \param arg A pointer to the context structure passed in the waveform_register_status_cb
/// \return 0 for success otherwise a negative value on error
static int echo_command(struct waveform_t *waveform __attribute__((unused)), unsigned int argc, char *argv[],
                        void *arg __attribute__((unused))) {
    fprintf(stderr, "Got a status for %s\n", argv[0]);
    fprintf(stderr, "Number of args is %u\n", argc);
    for(int i = 0; i < argc; ++i) {
        fprintf(stderr, "ARG #%u: %s\n", i, argv[i]);
    }
    return 0;
}

/// \brief A command callback to just print the arguments received.
/// This callback is used when the radio has received a
/// command destined for the waveform in the form "slice 1 waveform_cmd ..." where ... is filled in by freeform text
/// that's passed verbatim to the waveform.  The callback in libwaveform expects a "command" as it's first argument.
/// For example "slice 1 waveform_cmd set foo=bar"
/// \param waveform A pointer to the opaque waveform structure returned by waveform_create
/// \param argc The number of arguments in the waveform command
/// \param argv An array of the arguments in the waveform command
/// \param arg A pointer to the context structure passed in the waveform_register_command_cb
/// \return 0 for success otherwise a negative value on error
static int test_command(struct waveform_t *waveform __attribute__((unused)), unsigned int argc,
                        char *argv[], void *arg __attribute__((unused))) {
    for (int i = 0; i < argc; ++i)
        fprintf(stderr, "ARG #%u: %s\n", i, argv[i]);

    return 0;
}

/// \brief A callback function to process incoming receiver packets.
/// This is called once for every packet we receive from the
/// radio.  In this case we just clear out the samples we receive, replace them with the proper sine wave values, and
/// send it to the radio for the speaker data using waveform_send_data_packet.  We use the context passed to us that
/// we set in the registration command to keep track of our current phase and meter data.  After sending a packet we
/// update the meter data and send that to the radio as well.
/// \param waveform A pointer to the opaque waveform structure returned by waveform_create
/// \param packet A structure containing the VITA-49 packet data.  This should be considered read-only and opaque.
///               The various accessor functions should be used to access the data such as get_packet_len() used here.
/// \param packet_size The size of the waveform_vita_packet structure
/// \param arg A pointer to the context structure passed in the waveform_register_rx_data_cb
static void packet_rx(struct waveform_t *waveform, struct waveform_vita_packet *packet,
                      size_t packet_size __attribute__((unused)), void *arg __attribute__((unused))) {
    struct junk_context *ctx = waveform_get_context(waveform);

    if (ctx->tx) {
        return;
    }

    float null_samples[get_packet_len(packet)];
    memset(null_samples, 0, sizeof(null_samples));

    for (int i = 0; i < get_packet_len(packet); i += 2) {
        null_samples[i] = null_samples[i + 1] =
                          sin_table[ctx->rx_phase] * 0.5F;
        ctx->rx_phase = (ctx->rx_phase + 1) % 24;
    }

    waveform_send_data_packet(waveform, null_samples,
                              get_packet_len(packet), SPEAKER_DATA);

    waveform_meter_set_float_value(waveform, "junk-snr", (float) ctx->snr);
    waveform_meters_send(waveform);
    ctx->snr = ++ctx->snr > 100 ? -100 : ctx->snr;

    if (++ctx->byte_data_counter % 100 == 0) {
        size_t len = snprintf(NULL, 0, "Callback Counter: %ld\n", ctx->byte_data_counter);
        uint8_t data_message[len + 1];
        snprintf(data_message, sizeof(data_message), "Callback Counter: %ld\n", ctx->byte_data_counter);
        waveform_send_byte_data_packet(waveform, data_message, sizeof(data_message));
    }
}

/// \brief A callback function called when we receive a VITA-49 packet with data in it rather than samples.
/// This is used when a waveform is talking to a modem that performs the underlying modulation, such as the internal
/// RapidM modem on a 9000 series radio.
/// \param waveform A pointer to the opaque waveform structure returned by waveform_create
/// \param packet A structure containing the VITA-49 packet data.  This should be considered read-only and opaque.
///               The various accessor functions should be used to access the data.
/// \param packet_size The size of the waveform_vita_packet structure
/// \param arg A pointer to the context structure passed in the waveform_register_byte_data_cb
static void data_rx(struct waveform_t *waveform __attribute__((unused)),
                    struct waveform_vita_packet *packet, size_t packet_size __attribute__((unused)),
                    void *arg __attribute__((unused))) {
    fprintf(stderr, "Got packet...\n");
    fprintf(stderr, "  Length: %d\n", get_packet_byte_data_length(packet));
    fprintf(stderr, "  Content: %s\n", (char *) get_packet_byte_data(packet));
}

/// \brief A callback function called when we are in transmit mode and receive microphone data to transmit.
/// In this example we just replace out these samples with the sine wave data and send that to the radio.
/// \param waveform A pointer to the opaque waveform structure returned by waveform_create
/// \param packet A structure containing the VITA-49 packet data.  This should be considered read-only and opaque.
///               The various accessor functions should be used to access the data.
/// \param packet_size The size of the waveform_vita_packet structure
/// \param arg A pointer to the context structure passed in the waveform_register_tx_data_cb
static void packet_tx(struct waveform_t *waveform,
                      struct waveform_vita_packet *packet, size_t packet_size __attribute__((unused)),
                      void *arg __attribute__((unused))) {
    struct junk_context *ctx = waveform_get_context(waveform);

    if (false == ctx->tx) {
        return;
    }

    float xmit_samples[get_packet_len(packet)];
    memset(xmit_samples, 0, sizeof(xmit_samples));

    for (int i = 0; i < get_packet_len(packet); i += 2) {
        xmit_samples[i] = xmit_samples[i + 1] =
                          sin_table[ctx->tx_phase] * 0.5F;
        ctx->tx_phase = (ctx->tx_phase + 1) % 24;
    }

    waveform_send_data_packet(waveform, xmit_samples,
                              get_packet_len(packet), TRANSMITTER_DATA);
}

/// \brief A callback to be invoked on the completion of a command on the radio.  In this case we just print the
/// results of the command.
/// \param waveform A pointer to the opaque waveform structure returned by waveform_create
/// \param code The numeric error code returned from the command.  0 is success, anything else is a failure
/// \param message The descriptive message returned from the radio.  See the API documentation wiki
/// \param arg A pointer to the context structure passed in the waveform_send_api_command_cb
static void set_filter_callback(struct waveform_t *waveform __attribute__((unused)), const unsigned int code,
                                char *message, void *arg __attribute__((unused))) {
    fprintf(stderr, "Invoked callback for code %d, message %s\n", code,
            message);
}

/// \brief A callback to be called when the waveform changes state.  It is important to implement this callback so that
/// your waveform knows when we have keyed the transmitter and we should start sending TX data packets rather than
/// speaker packets.  In this function we note in the context structure that we are in transmit mode and allow the
/// callbacks to do the correct thing.
/// \param waveform A pointer to the opaque waveform structure returned by waveform_create
/// \param state The new state of the waveform
/// \param arg A pointer to the context structure passed in the waveform_register_state_cb
static void state_test(struct waveform_t *waveform, const enum waveform_state state,
                       void *arg __attribute__((unused))) {
    struct junk_context *ctx = waveform_get_context(waveform);

    switch (state) {

        // Active state is when the user has selected the waveform in the user interface indicating their intent to use
        // this waveform.  We do any preparation we need to do to be able to receive data such as reinitializing data
        // structures, clearing buffers, etc.  In our case here we need to tell the radio to set the filter width to
        // 3000 Hz.
        case ACTIVE:
            fprintf(stderr, "wf is active\n");
            waveform_send_api_command_cb(waveform, &set_filter_callback,
                                         NULL, "filt 0 100 3000");
            break;

        // Inactive state is when the user has selected another mode on the radio user interface.  We need to do any
        // cleanup here.  Remember that the user may not select this waveform again for a long time, so we shouldn't
        // keep any large chunks of memory around or be running unnecessary code.  This should be considered a request
        // to "sleep" the waveform.
        case INACTIVE:
            fprintf(stderr, "wf is inactive\n");
            break;

        // PTT requested is the state triggered when the user keys the radio, whether via MOX, the PTT button on the
        // microphone or VOX.  When we receive this state we must make preparations to cease sending data to the radio
        // for the speaker and prepare to send a transmit stream.  In our case here we set the tx variable in the
        // context structure to true which causes the receive packet callback to be a noop and the transmit packet
        // callback to start sending TX data.
        case PTT_REQUESTED:
            fprintf(stderr, "ptt requested\n");
            ctx->tx = true;
            break;

        // Unkey requested is the state triggered with the user unkeys the radio, whether via MOX, the PTT button on the
        // microphone or VOX.  When we receive this state we must make preparations to cease sending TX data packets to
        // the radio and begin processing the receive streams.  In our case here we set the tx variable in the context
        // structure to false, which causes the receive packet callback to start sending speaker data and the transmit
        // callback to be a noop.
        case UNKEY_REQUESTED:
            fprintf(stderr, "unkey requested\n");
            ctx->tx = false;
            break;
        default:
            fprintf(stderr, "unknown state received");
            break;
    }
}

/// \brief Print a usage message to the console
/// @param progname The name of this program
static void usage(const char *progname) {
    fprintf(stderr, "Usage: %s [options]\n\n", progname);
    fprintf(stderr, "Options:\n");
    fprintf(stderr, "  -h <hostname>, --host=<hostname>  Hostname or IP of the radio [default: perform discovery]\n");
}

/// \brief The command line parameters.  Currently just the hostname or IP of the radio
static const struct option example_options[] = {
    {
        .name = "host",
        .has_arg = required_argument,
        .flag = NULL,
        .val = 'h' //  This keeps the value the same as the short value -h
    },
    {0} // Sentinel
};

// ****************************************
// Global Functions
// ****************************************
int main(const int argc, char **argv) {
    struct sockaddr_in *addr = NULL;

    // Create an instance of the waveform context structure to register with the library.  We can get a pointer to
    // this structure back by using waveform_get_context on the opaque waveform structure.  The library user is
    // responsible for all memory management and thread concurrency issues with this structure. The library merely
    // stores a pointer and regurgitates it back to the user when asked.
    struct junk_context ctx = {0};

    // Parse the command line
    while (1) {
        int indexptr;
        const int option = getopt_long(argc, argv, "h:", example_options, &indexptr);

        if (option == -1) // We're done with options
            break;

        switch (option) { // NOLINT(*-multiway-paths-covered)
            case 'h': {
                if (addr != NULL) {
                    free(addr);
                    usage(basename(argv[0]));
                    exit(1);
                }

                struct addrinfo *addrlist;
                const int ret = getaddrinfo(optarg, "4992", NULL, &addrlist);
                if (ret != 0) {
                    fprintf(stderr, "Host lookup for %s failed: %s\n", optarg, gai_strerror(ret));
                    exit(1);
                }

                addr = malloc(sizeof(*addr));
                memcpy(addr, addrlist[0].ai_addr, sizeof(*addr));

                freeaddrinfo(addrlist);
                break;
            }
            default:
                usage(basename(argv[0]));
                exit(1);
        }
    }

    if (optind < argc) {
        fprintf(stderr, "Non option elements detected:");
        for (size_t i = optind; i < argc; ++i) {
            fprintf(stderr, " %s", argv[i]);
        }
        fprintf(stderr, "\n");
        usage(basename(argv[0]));
        exit(1);
    }

    //  If we didn't get an address on the command line, perform discovery for it.  We wait for 10 seconds before giving
    //  up.  Most of the time a production waveform will not perform this process as it should be given the IP address
    //  to the local radio from some process.
    if (addr == NULL) {
        const struct timeval timeout = {
            .tv_sec = 10,
            .tv_usec = 0
        };

        addr = waveform_discover_radio(&timeout);
        if (addr == NULL) {
            fprintf(stderr, "No radio found");
            return 0;
        }
    }

    fprintf(stderr, "Connecting to radio at %s:%u\n", inet_ntoa(addr->sin_addr), ntohs(addr->sin_port));

    // Create a radio to which to connect.  We need its address in order to create an instance.  We are returned an
    // opaque structure to manage the radio.  Note that this is just a data structure at this point and we have not
    // connected to the radio.  The library does not connect to the radio until waveform_radio_start is invoked.
    struct radio_t *radio = waveform_radio_create(addr);
    free(addr);


    // Create a waveform on the radio.  We need a name for it, which the radio uses internally to track the waveform.
    // The short name is the name that will appear on the radio UI when selecting the "mode."  It must be four
    // characters or less to fit within the confines of the GUI.

    // The underlying mode determines what demodulation
    // is done by the radio before sending the sample data to the waveform.  For example, if you are attempting to
    // write a waveform implementing 1200baud AFSK, you would like to use "FM" as your underlying mode to be able to
    // have the tones already decoded.  Conversely, you would want to use something like DIGU to decode HF digital
    // modes.  There is a special mode called "RAW" that is not presented to users on the UI, but is nonetheless
    // present for waveforms.  This will give you unmodulated data as I/Q pairs instead of L/R baseband data.  In this
    // way you can do anything you want with it.
    struct waveform_t *test_waveform =
            waveform_create(radio, "JunkMode", "JUNK", "DIGU", "1.0.0", SR_24K);

    // Register a status callback so that we get updates on the slice.  Any "slice" status will cause the echo_command
    // callback to be run.  You can specify a pointer to a context structure if you need to pass something just for
    // this callback, otherwise you can use the global waveform context structure.  Note that this command does not
    // cause the library to subscribe to these status messages; it only sets up a callback in case it hears one.  You
    // must use waveform_send_api_command to send a subscribe command to the radio as per the API.  See the Wiki at
    // https://github.com/flexradio/smartsdr-api-docs/wiki/TCPIP-sub for more information on the subscription types
    int res = waveform_register_status_cb(test_waveform, "slice", echo_command, NULL);
    if (res == -1) {
        fprintf(stderr, "Failed to register status callback\n");
    }

    // Register a state callback for the waveform.  This callback is called when the waveform is activated/deactivated
    // and when PTT is asserted or deasserted.  See the state_test callback in this file for a more detailed description
    // of the states and their usages.
    res = waveform_register_state_cb(test_waveform, &state_test, NULL);
    if (res == -1) {
        fprintf(stderr, "Failed to register state callback\n");
    }

    // Register a callback to handle receiver data.  Whenever a receiver VITA-49 packet is received, this callback is
    // fired.  The waveform is expected to process whatever data it's given from the receiver and act appropriately.
    // If audio is desired from the speaker or remote audio, the waveform must send packets back to the radio on the
    // speaker stream.  The pacing of such packets should match the incoming receiver data, i.e. you should be sending
    // out as many samples as you receive.
    res = waveform_register_rx_data_cb(test_waveform, &packet_rx, NULL);
    if (res == -1) {
        fprintf(stderr, "Failed to register RX data callback\n");
    }

    // Register a callback to handle transmitter data.  Whenever a transmit VITA-49 packet is received, this callback
    // is fired.  This packet will contain data from the microphone for the waveforms use.  If the waveform is not
    // using the microphone for this mode (i.e. data), it is free to ignore the payload.  However, these packets can
    // be used for pacing the data stream.  For every sample we get from the microphone, a sample is needs to be
    // generated to the transmitter output stream from the waveform.  You need to use waveform_send_data_packet to
    // send samples to the radio to be transmitted.  Note that the transmitter will not key until it has started to
    // receive transmitter stream packets and it will not unkey until it has ceased to receive those packets.
    res = waveform_register_tx_data_cb(test_waveform, &packet_tx, NULL);
    if (res == -1) {
        fprintf(stderr, "Failed to register TX data callback\n");
    }

    // Register a callback for "byte stream" data.  This is data routed through the radio from a byte stream source
    // such as a serial port or a RapidM modem.
    res = waveform_register_byte_data_cb(test_waveform, &data_rx, NULL);
    if (res == -1) {
        fprintf(stderr, "Failed to register byte data callback\n");
    }

    // Register a callback for commands from the client.  This callback is called whenever a command is received that
    // is needed to be processed by the waveform.  This functionality can be used to, for example, set a submode that
    // this waveform handles.  In the FreeDV waveform, we use this to determine whether to use 1600, 700C or any of
    // the other sub modes.  This can also be used to set any other internal parameters of the waveform.  There is
    // currently no generic way in the client software to send these commands, but they can be sent using a companion
    // application to control the waveform.
    res = waveform_register_command_cb(test_waveform, "set", test_command, NULL);
    if (res == -1) {
        fprintf(stderr, "Failed to register command callback\n");
    }

    // Set up the meters we intend to send to the radio.  This sends a command to make sure all of those meters are
    // registered and ready to receive data.  The data can then be sent at periodic intervals using the
    // waveform_meter_set_*_value family of functions followed by waveform_meters_send.
    waveform_register_meter_list(test_waveform, meters, ARRAY_SIZE(meters));

    // Set the waveform context.  This is a pointer to a data structure of your choice that's kept with the opaque
    // waveform structure and made available to any of the callbacks using the waveform_get_context() function.  This
    // can be used to store any sort of persistent state you need to have between callbacks.  In our example here we
    // use this to store the current phase of the NCOs and whether we are transmitting.  This could also be used to
    // store our current submode or any other parameters.  There is also a per-callback context in case you need state
    // data that only applies to a single callback.
    waveform_set_context(test_waveform, &ctx);

    // Start the radio.  This causes the library to connect to the radio and start its various event loops.  It is not
    // currently supported to change any callbacks after the waveform_start_radio command has been executed.
    res = waveform_radio_start(radio);
    if (res == -1) {
        fprintf(stderr, "Failed to start radio\n");
    }

    // Wait for the radio to be finished.  In normal operation we should not ever get here unless the radio is going
    // to shut down for some reason or we have been forcibly disconnected by the radio.  Essentially this waits until
    // the various event loop threads have ceased running.
    res = waveform_radio_wait(radio);
    if (res == -1) {
        fprintf(stderr, "Failed to wait on radio completion\n");
    }
}
