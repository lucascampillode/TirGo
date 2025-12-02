#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool
import actionlib
from pal_interaction_msgs.msg import TtsAction, TtsGoal, TtsText


class TirgoSpeechNode:
    """
    Nodo de alto nivel para la voz de TIAGo.

    - Suscribe a /tirgo/say (String): cualquier texto que llegue, TIAGo lo dice.
    - Suscribe a /tirgo/tiago/delivered (Bool): cuando se entrega el envase,
      TIAGo dice una frase de despedida y publica /tirgo/tiago/farewell_done = True.
    - Suscribe a /tirgo/mission/start (String): para resetear estado al inicio
      de cada misión.
    """

    def __init__(self):
        rospy.init_node("tirgo_speech_node")

        # Publisher de la flag final de despedida
        self.pub_farewell = rospy.Publisher(
            "/tirgo/tiago/farewell_done", Bool, queue_size=1
        )

        # Cliente TTS de TIAGo
        self.tts_client = actionlib.SimpleActionClient("/tts", TtsAction)

        self.tts_available = False
        rospy.loginfo("[SPEECH] Esperando al action /tts...")
        if self.tts_client.wait_for_server(rospy.Duration(5.0)):
            self.tts_available = True
            rospy.loginfo("[SPEECH] /tts listo.")
        else:
            rospy.logwarn("[SPEECH] No se encontró /tts en 5s. El nodo no hablará, solo hará logs.")

        # Topic genérico de texto a decir
        self.sub_say = rospy.Subscriber(
            "/tirgo/say", String, self.cb_say, queue_size=10
        )

        # Entrega al paciente -> disparar despedida
        self.sub_delivered = rospy.Subscriber(
            "/tirgo/tiago/delivered", Bool, self.cb_delivered, queue_size=10
        )

        # Inicio de misión -> reseteamos el flag interno
        self.sub_mission_start = rospy.Subscriber(
            "/tirgo/mission/start", String, self.cb_mission_start, queue_size=10
        )

        # Flag interno: ¿ya hemos hecho la despedida en esta misión?
        self._farewell_done = False

    # ----------------------------
    # Helpers internos
    # ----------------------------
    def _say(self, text: str, lang: str = "es_ES"):
        text = (text or "").strip()
        if not text:
            return

        if not self.tts_available:
            rospy.loginfo(f"[SPEECH] (sin /tts) diría: {text}")
            return

        goal = TtsGoal()
        goal.rawtext = TtsText(text=text, lang_id=lang)
        goal.speakerName = ""
        goal.wait_before_speaking = 0.0

        self.tts_client.send_goal(goal)
        rospy.loginfo(f"[SPEECH] TIAGo diciendo: {text}")

    # ----------------------------
    # Callbacks
    # ----------------------------
    def cb_say(self, msg: String):
        """
        Cualquier nodo puede publicar en /tirgo/say para que TIAGo hable.
        """
        self._say(msg.data)

    def cb_mission_start(self, msg: String):
        """
        Cada vez que se inicia una misión nueva, reseteamos el estado de despedida.
        """
        self._farewell_done = False
        rospy.loginfo("[SPEECH] Misión nueva -> _farewell_done = False")

    def cb_delivered(self, msg: Bool):
        """
        Cuando el robot confirma la entrega al paciente, lanzamos la despedida
        (solo una vez por misión) y publicamos /tirgo/tiago/farewell_done = true.
        """
        if not msg.data:
            return

        if self._farewell_done:
            rospy.loginfo("[SPEECH] delivered recibido, pero la despedida ya se hizo en esta misión.")
            return  # ya hicimos la despedida en esta misión

        self._farewell_done = True
        rospy.loginfo("[SPEECH] Entrega confirmada -> lanzando despedida...")

        # Frase de despedida (ajústala a tu gusto)
        self._say("He terminado con la entrega. Muchas gracias, hasta luego.")

        # Pequeña espera opcional para no ir más rápido que el audio
        rospy.sleep(1.0)

        # Flag para el TirgoMissionServer
        self.pub_farewell.publish(Bool(data=True))
        rospy.loginfo("[SPEECH] Publicado /tirgo/tiago/farewell_done = true")


def main():
    try:
        node = TirgoSpeechNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
