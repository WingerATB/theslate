# A leaf on purpose: it depends on nothing in this tree, so both the BLE
# transport (which must recognise a GoPro advert) and the GoPro session (which
# must speak the protocol) can depend on it without a cycle.
idf_component_register(INCLUDE_DIRS "include")
