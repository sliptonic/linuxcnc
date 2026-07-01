# Optional MQTT transport following the standard MTConnect MQTT binding.
#
# Publishes the standard MTConnect response documents to the standard topics:
#   <prefix>/Probe/<uuid>              (retained)
#   <prefix>/Current/<uuid>           (at the sample interval)
#   <prefix>/Sample/<uuid>            (when new observations arrive)
#   <prefix>/Asset/<uuid>/<assetId>   (on asset change)
#
# Reuses paho-mqtt, already used by src/hal/user_comps/mqtt-publisher.py.


class MqttAgent:
    def __init__(self, agent, broker="localhost", port=1883,
                 prefix="MTConnect", username=None, password=None):
        try:
            import paho.mqtt.client as mqtt
        except ModuleNotFoundError:
            print("error: Missing Python module paho.")
            print("error: On Debian, run 'sudo apt install python3-paho-mqtt'.")
            raise
        self.agent = agent
        self.prefix = prefix.rstrip("/")
        self.uuid = agent.config.uuid
        self._last_sample_seq = 1

        self.client = mqtt.Client()
        if username:
            self.client.username_pw_set(username, password)
        self.client.on_connect = lambda c, u, f, rc: self._on_connect()
        self.client.connect_async(broker, port, 60)
        self.client.loop_start()

    def _topic(self, kind, suffix=None):
        base = "%s/%s/%s" % (self.prefix, kind, self.uuid)
        return "%s/%s" % (base, suffix) if suffix else base

    def _on_connect(self):
        # Publish the (retained) device model so late subscribers can decode.
        self.publish_probe()

    def publish_probe(self):
        self.client.publish(self._topic("Probe"), self.agent.probe_document(),
                            retain=True)

    def publish_current(self):
        self.client.publish(self._topic("Current"), self.agent.current_document())

    def publish_sample(self):
        first, nxt = self.agent.buffer.first_sequence, self.agent.buffer.next_sequence
        if nxt <= self._last_sample_seq:
            return
        start = max(self._last_sample_seq, first)
        doc = self.agent.sample_document(start, nxt - start)
        self.client.publish(self._topic("Sample"), doc)
        self._last_sample_seq = nxt

    def publish_assets(self):
        for asset in self.agent.source.tool_assets():
            self.client.publish(self._topic("Asset", asset.asset_id),
                                self.agent.assets_document(), retain=True)

    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()
