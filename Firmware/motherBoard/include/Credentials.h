#ifndef _CREDENTIALS_
#define _CREDENTIALS_

#define THINGSBOARD_SERVER "mon.medicalopenworld.org"
#define THINGSBOARD_PORT 1883

#define FACTORY_SERVER 0
#define DEMO_SERVER 1

#define THINGSBOARD_PROVISION_SERVER FACTORY_SERVER

#if(THINGSBOARD_PROVISION_SERVER == DEMO_SERVER)
#define PROVISION_DEVICE_KEY "bztump0738iuc2ggreix"
#define PROVISION_DEVICE_SECRET "0znp47gkyh9hbljq1opm"
#elif (THINGSBOARD_PROVISION_SERVER == FACTORY_SERVER)
#define PROVISION_DEVICE_KEY "8f9yvlkqxirz2pq9n5co"
#define PROVISION_DEVICE_SECRET "bz9fwzi8t3pnqxdlipqz"
#endif

#define ssid "in3wifi"
#define wifiPassword "12345678"
#endif // _CREDENTIALS_
