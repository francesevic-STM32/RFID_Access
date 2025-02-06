/*
 * poruke.c
 *
 *  Created on: Feb 1, 2025
 *      Author: danijel.francesevic
 */


#ifndef INC_PORUKE_C_
#define INC_PORUKE_C_

#include "poruke.h"

// Denied messages
const char *denied_messages[][2] = {
    {"NO ENTRY", "FOR YOU!"},
    {"ACCESS DENIED", "NICE TRY"},
    {"GO AWAY", "HUMAN!"},
    {"ERROR 403:", "FORBIDDEN"},
    {"SORRY,", "WRONG CARD"},
    {"NOT TODAY!", "TRY TOMORROW"},
    {"DENIED.", "CRY ABOUT IT."},
    {"TRY AGAIN", "...IN 10 YEARS"},
    {"ERROR 404", "CARD NOT FOUND"},
    {"STOP!", "WHO GOES THERE?"},
    {"NICE TRY!", "HUMAN!"},
    {"YOUR CARD", "IS SUS"},
    {"INVALID ENTRY", "TRY MAGIC?"},
    {"ACCESS BLOCKED", "GO HOME"},
    {"CARD? MORE LIKE", "SCAM!"},
    {"DENIED, CALL", "TECH SUPPORT"},
    {"WRONG CARD", "TRY YOUR LUCK"},
    {"SORRY,", "I DONT KNOW YOU"},
    {"ACCESS DENIED", "NICE PASSWRD THO"},
    {"ACCESS DENIED!", "WOMP! WOMP!"},
    {"ACCESS DENIED", "Nah, try again."},
    {"DOOR STATUS:", "Still closed."},
    {"WRONG CARD", "This ain't it!"},
    {"ACCESS DENIED", "Not today, my guy."},
    {"LOGIN FAILED", "Check yourself."},
    {"SECURITY ALERT", "That ain't valid."},
    {"ACCESS BLOCKED", "Nice try, FBI."},
    {"CARD REJECTED", "Bruh, really?"},
    {"DOOR SAYS NO", "Maybe next time."},
    {"INVALID ENTRY", "This ain't the way."},
    {"ACCESS DENIED", "Try turning it off."},
    {"SECURITY CHECK", "You ain't on the list."},
    {"WRONG KEYCARD", "Not in this lifetime."},
    {"BAD CREDENTIALS", "Try another one."},
    {"ACCESS DENIED", "Ask nicely"},
    {"DENIED", "Maybe with a bribe?"},
    {"AUTHORIZATION:", "Yeah, no."},
    {"LOGIN FAILED", "Are you trying?"}
};
const uint8_t denied_messages_count = sizeof(denied_messages) / sizeof(denied_messages[0]);

// Granted messages
const char *granted_messages[][2] = {
    {"ACCESS", "GRANTED!"},
    {"DOOR IS OPEN", "WELCOME!"},
    {"YOU MAY", "ENTER!"},
    {"HELLO,", "COME IN!"},
    {"VIP PASS", "ACCEPTED!"},
    {"GRANTED!", "DON'T BREAK IT!"},
    {"WELCOME,", "DON'T TOUCH STUFF!"},
    {"ACCESS OK,", "BEHAVE!"},
    {"GREEN LIGHT!", "GO GO GO!"},
    {"SECURITY CHECK", "PASSED!"},
    {"ENTRY", "APPROVED!"},
    {"DOORS", "UNLOCKED!"},
    {"WELCOME,", "DON'T STEAL!"},
    {"YOU DID IT!", "NOW RUN!"},
    {"SCAN OK,", "WELCOME!"},
    {"NICE CARD,", "COME IN!"},
    {"SYSTEM SAYS:", "YES!"},
    {"GRANTED,", "LUCKY YOU!"},
    {"ACCESS OPEN,", "GOOD DAY!"},
    {"WELCOME,", "ENJOY YOUR STAY!"},
    {"WELCOME! Make", "yourself at home"},
    {"SECURITY CHECK:", "YOU PASSED!"},
    {"ENTRY APPROVED!", "ACT NATURAL!"},
    {"YOU'RE IN!", "DONT CAUSE CHAOS!"},
    {"WELCOME! Now pay", "the entrance fee!"},
    {"ALLOWED! No", "fun allowed, tho!"},
    {"YOU'RE IN!", "Dont tell others"},
    {"DOOR OPENED!", "BOLDLY GO!"},
    {"GRANTED!    Do I", "detect snacks?"},
    {"RFID SCAN OK!", "FASCINATING!"},
    {"DOOR STATUS:", "100 percent OPEN"},
    {"ACCESS GRANTED", "Like a boss."},
    {"YOU SHALL PASS!", " "},
    {"ACCESS GRANTED", "Big brain move."},
    {"DOOR IS OPEN", "No cap."},
    {"CONGRATS, NERD!", "Enjoy your access."},
    {"WELCOME, HUMAN", "AI overlord approves."},
    {"ACCESS GRANTED", "Time to flex."},
    {"KEYCARD ACCEPTED", "Welcome aboard."},
    {"LOGIN SUCCESS", "Admin privileges."},
    {"DOOR UNLOCKED", "But at what cost?"},
    {"AUTH PASSED", "System online."},
    {"NICE TRY FBI", "You're in."},
    {"SECURITY CHECK", "You made it."},
    {"ACCESS OK", "Proceed carefully."},
    {"PASSWORD: OK", "Enjoy the access."}
};
const uint8_t granted_messages_count = sizeof(granted_messages) / sizeof(granted_messages[0]);

// Welcome messages
const char *welcome_messages[][2] = {
    {"RFID system says:", "Try your luck!"},
    {"No card?", "No entry!"},
    {"Scanning for ID...", " "},
    {"Waiting for a ", "card... "},
    {"Identify ", "yourself"},
    {"Your move...", "Tap that card!"},
    {"Insert card", "Not coins!"},
    {"Tap the card...", " "},
    {"Insert card...", "Not an SD one!"},
    {"Lets see...Are", "you in database?"}
};
const uint8_t welcome_messages_count = sizeof(welcome_messages) / sizeof(welcome_messages[0]);

#endif // MESSAGES_H
