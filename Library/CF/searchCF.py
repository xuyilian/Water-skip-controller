import cflib.crtp  # noqa


def search_cf(cf):
    # Initialize the low-level drivers (don't list the debug drivers)
    cf.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cf.scan_interfaces()
    if len(available) > 0:
        print('Crazyflies found:')
        for i in available:
            print(i[0])
    else:
        print('No Crazyflies found')
    return available
