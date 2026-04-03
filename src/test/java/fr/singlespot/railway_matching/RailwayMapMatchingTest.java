package fr.singlespot.railway_matching;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import de.geofabrik.railway_routing.http.RailwayRoutingApplication;
import de.geofabrik.railway_routing.http.RailwayRoutingServerConfiguration;
import io.dropwizard.testing.DropwizardTestSupport;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.CsvSource;

import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Test class for RailwayMapMatching using real GPX data
 */
public class RailwayMapMatchingTest {

    // Full GPX data provided by the user
    private static final String FULL_GPX_DATA = """
            <?xml version="1.0" encoding="UTF-8"?>
            <gpx xmlns="http://www.topografix.com/GPX/1/1" xmlns:gpxtpx="http://www.garmin.com/xmlschemas/TrackPointExtension/v1" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd" version="1.1" creator="gpx.py -- https://github.com/tkrajina/gpxpy">
              <trk>
                <trkseg>
                  <trkpt lat="48.84190384374861" lon="2.36665762538408">
                    <time>2023-05-26T21:06:11+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>76</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.839635401554126" lon="2.3685173856590476">
                    <time>2023-05-26T21:06:34+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>800</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.83750478433458" lon="2.370129556367319">
                    <time>2023-05-26T21:06:58+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>960</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.83494767468244" lon="2.371895005949924">
                    <time>2023-05-26T21:07:30+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>2000</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.83470570459887" lon="2.3720549239508246">
                    <time>2023-05-26T21:07:55+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>2000</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.83459373344988" lon="2.372134821245617">
                    <time>2023-05-26T21:09:00+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>2000</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.83456723234492" lon="2.37215677181476">
                    <time>2023-05-26T21:09:15+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>2000</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.83876684932885" lon="2.373234499567417">
                    <time>2023-05-26T21:12:58+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>2000</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.83897812626875" lon="2.3714777775118403">
                    <time>2023-05-26T21:13:36+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>2000</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.83585517123431" lon="2.3692976537318473">
                    <time>2023-05-26T21:14:00+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>2000</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.83468678309662" lon="2.371931302109908">
                    <time>2023-05-26T21:14:28+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>2000</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.83463126222878" lon="2.3720641868292947">
                    <time>2023-05-26T21:14:48+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>2000</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.83356031034964" lon="2.372807906942789">
                    <time>2023-05-26T21:15:11+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>50</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.83341632265178" lon="2.3733140597323157">
                    <time>2023-05-26T21:15:21+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>28</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.83304438961556" lon="2.3734730607400114">
                    <time>2023-05-26T21:15:30+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>30</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.833044389615544" lon="2.3734730607400105">
                    <time>2023-05-26T21:15:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>278</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.83119624885266" lon="2.3753100065665036">
                    <time>2023-05-26T21:16:41+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>459</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.822067455354194" lon="2.383977469796847">
                    <time>2023-05-26T21:17:43+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.795788667022826" lon="2.4051919770675823">
                    <time>2023-05-26T21:20:21+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>16</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.76785631509046" lon="2.4108180996675697">
                    <time>2023-05-26T21:22:48+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>17</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.73710639845238" lon="2.4271220988181907">
                    <time>2023-05-26T21:25:22+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>21</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.70142610807864" lon="2.3939641522363746">
                    <time>2023-05-26T21:27:48+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>17</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.675587329147426" lon="2.351584775382746">
                    <time>2023-05-26T21:30:22+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>17</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.64666309954341" lon="2.305778176359686">
                    <time>2023-05-26T21:32:48+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>25</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.60683601745465" lon="2.3020923292950863">
                    <time>2023-05-26T21:35:23+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.57582226496271" lon="2.2899248616122314">
                    <time>2023-05-26T21:37:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.52850525137187" lon="2.279503899802134">
                    <time>2023-05-26T21:40:24+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.515030733971486" lon="2.231017996444395">
                    <time>2023-05-26T21:42:48+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>43</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.49220782029519" lon="2.193512490116319">
                    <time>2023-05-26T21:45:25+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.46355781680735" lon="2.1756006764476603">
                    <time>2023-05-26T21:47:38+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.43135531610459" lon="2.146263662196008">
                    <time>2023-05-26T21:50:25+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.40199368617443" lon="2.095907979251637">
                    <time>2023-05-26T21:52:48+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.3345836451857" lon="2.017688022661842">
                    <time>2023-05-26T21:56:30+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.31058153398324" lon="2.003054351296043">
                    <time>2023-05-26T21:57:38+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.22454435188583" lon="1.9561495055207339">
                    <time>2023-05-26T22:01:31+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.19968615591391" lon="1.9431381324068508">
                    <time>2023-05-26T22:02:38+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.11329198777983" lon="1.8979932120026577">
                    <time>2023-05-26T22:06:31+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.08873323558625" lon="1.8868184618338013">
                    <time>2023-05-26T22:07:38+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.04394943847888" lon="1.8790275750618992">
                    <time>2023-05-26T22:11:32+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>27</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.026502766955716" lon="1.8791554875470176">
                    <time>2023-05-26T22:12:48+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>71</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.02422210776027" lon="1.8792411237511983">
                    <time>2023-05-26T22:12:58+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>72</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.02163879300618" lon="1.8791083135322921">
                    <time>2023-05-26T22:13:08+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>95</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.01954440738244" lon="1.879231771005892">
                    <time>2023-05-26T22:13:18+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>80</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.017186542242236" lon="1.879284815357046">
                    <time>2023-05-26T22:13:29+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>99</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.01486544885095" lon="1.8795809422133785">
                    <time>2023-05-26T22:13:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>38</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.97783684932087" lon="1.8862203253970202">
                    <time>2023-05-26T22:16:35+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>24</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.96448300985829" lon="1.888825945450284">
                    <time>2023-05-26T22:17:38+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.9352366212908" lon="1.902008007361228">
                    <time>2023-05-26T22:21:35+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.930344119525905" lon="1.9044516652980548">
                    <time>2023-05-26T22:22:48+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>25</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.9274485986146" lon="1.905830807631521">
                    <time>2023-05-26T22:26:36+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.92743599049502" lon="1.905876556907104">
                    <time>2023-05-26T22:27:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.91330330170996" lon="1.8841143777229652">
                    <time>2023-05-26T22:31:36+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.90529285524007" lon="1.8603339932528105">
                    <time>2023-05-26T22:32:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>28</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.880657994490086" lon="1.7926136269798287">
                    <time>2023-05-26T22:36:37+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.8747470349961" lon="1.7759772909713112">
                    <time>2023-05-26T22:37:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.84070587405792" lon="1.706506921744447">
                    <time>2023-05-26T22:41:37+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.82759113217382" lon="1.6887882387074336">
                    <time>2023-05-26T22:42:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>19</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.784681963025704" lon="1.6337915022972025">
                    <time>2023-05-26T22:46:38+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.77332357250887" lon="1.6190380782188558">
                    <time>2023-05-26T22:47:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.734367717650805" lon="1.5546070399603902">
                    <time>2023-05-26T22:51:38+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.72553507698736" lon="1.53928946352619">
                    <time>2023-05-26T22:52:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.69018541397626" lon="1.4744537257671488">
                    <time>2023-05-26T22:56:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.68185178664481" lon="1.4563734182597925">
                    <time>2023-05-26T22:57:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.637422896382894" lon="1.3952188023881238">
                    <time>2023-05-26T23:01:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.625306209713045" lon="1.3753781352878982">
                    <time>2023-05-26T23:02:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.56802823938608" lon="1.3050094286733684">
                    <time>2023-05-26T23:06:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.55165297873584" lon="1.2877546081783933">
                    <time>2023-05-26T23:07:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.49568323610986" lon="1.2010386093322853">
                    <time>2023-05-26T23:11:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.488703388330435" lon="1.1740490678259237">
                    <time>2023-05-26T23:12:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.458321722313855" lon="1.0695348327318277">
                    <time>2023-05-26T23:16:41+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.44895293847745" lon="1.0469556408654976">
                    <time>2023-05-26T23:17:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.41746985232306" lon="0.9685513964287615">
                    <time>2023-05-26T23:21:41+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.4157762169075" lon="0.9526271016385093">
                    <time>2023-05-26T23:22:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.408542093279216" lon="0.885557902421402">
                    <time>2023-05-26T23:26:42+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.406774004306804" lon="0.8693036309680995">
                    <time>2023-05-26T23:27:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.38732925256107" lon="0.8115333455456415">
                    <time>2023-05-26T23:31:43+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.38834741655514" lon="0.7927273652826362">
                    <time>2023-05-26T23:32:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.382131989062984" lon="0.7151428492595511">
                    <time>2023-05-26T23:36:44+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.37540405425423" lon="0.7036540526723887">
                    <time>2023-05-26T23:37:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.33477057617504" lon="0.6687111330939527">
                    <time>2023-05-26T23:41:44+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.322205007377065" lon="0.6653958037640932">
                    <time>2023-05-26T23:42:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.26545951268823" lon="0.6504397774325162">
                    <time>2023-05-26T23:46:44+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>17</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.25167188709356" lon="0.6468553995801773">
                    <time>2023-05-26T23:47:43+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.194939420556686" lon="0.6318176718370079">
                    <time>2023-05-26T23:51:44+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.178046558774554" lon="0.6273792590814792">
                    <time>2023-05-26T23:52:41+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.097689772470204" lon="0.5784350850293679">
                    <time>2023-05-26T23:56:45+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="47.07687579304844" lon="0.580611688921256">
                    <time>2023-05-26T23:57:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.98550069167703" lon="0.6022792484057918">
                    <time>2023-05-27T00:01:46+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.96568967319716" lon="0.6091694628705456">
                    <time>2023-05-27T00:02:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.87664182776778" lon="0.5706992792726223">
                    <time>2023-05-27T00:06:47+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.858604615422614" lon="0.5581190520266629">
                    <time>2023-05-27T00:07:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.77224444973014" lon="0.5150499924981821">
                    <time>2023-05-27T00:11:48+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.75803059794884" lon="0.49487583840964045">
                    <time>2023-05-27T00:12:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.69226120016901" lon="0.3935263467926699">
                    <time>2023-05-27T00:17:01+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>28</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.67690984275911" lon="0.3792799620878807">
                    <time>2023-05-27T00:17:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>21</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.600852144488485" lon="0.34365566843804896">
                    <time>2023-05-27T00:22:08+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>29</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.59246341186444" lon="0.33985305399772825">
                    <time>2023-05-27T00:22:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.52003904630048" lon="0.33330680765631227">
                    <time>2023-05-27T00:27:34+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>27</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.51446916220373" lon="0.33178988630498146">
                    <time>2023-05-27T00:27:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>45</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.45875186798806" lon="0.2997574422595945">
                    <time>2023-05-27T00:32:35+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>19</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.458364508320685" lon="0.2994286498286187">
                    <time>2023-05-27T00:32:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>23</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.42309552001521" lon="0.2617767818545931">
                    <time>2023-05-27T00:37:35+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>19</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.41989627401851" lon="0.25783932322394804">
                    <time>2023-05-27T00:37:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>66</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.41761393822394" lon="0.25489385755566085">
                    <time>2023-05-27T00:37:59+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>72</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.41503873615989" lon="0.25308992176917083">
                    <time>2023-05-27T00:38:09+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>77</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.41146686569006" lon="0.25115301148289215">
                    <time>2023-05-27T00:38:19+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>32</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.31611474048839" lon="0.24543418403849598">
                    <time>2023-05-27T00:42:43+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>339</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.305145171665586" lon="0.2447289543217719">
                    <time>2023-05-27T00:43:09+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>44</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.19636215586769" lon="0.23242509446575">
                    <time>2023-05-27T00:47:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>53</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.19149341416531" lon="0.23190837592714014">
                    <time>2023-05-27T00:48:01+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>20</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.08411346185494" lon="0.22310959674969444">
                    <time>2023-05-27T00:52:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>22</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.07919995823145" lon="0.22499159161360793">
                    <time>2023-05-27T00:53:01+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.05136547777313" lon="0.21328158793100913">
                    <time>2023-05-27T00:57:48+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.01132914136815" lon="0.16291361472788107">
                    <time>2023-05-27T01:02:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="46.0086657522179" lon="0.15559748580807603">
                    <time>2023-05-27T01:03:03+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.91669633463726" lon="0.1267393366392279">
                    <time>2023-05-27T01:07:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>24</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.91102226685693" lon="0.12477133045237598">
                    <time>2023-05-27T01:08:04+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.81569961228426" lon="0.14643546920742287">
                    <time>2023-05-27T01:12:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>19</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.81279859882416" lon="0.14943935908391087">
                    <time>2023-05-27T01:13:05+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>22</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.71193651548384" lon="0.1582614687344329">
                    <time>2023-05-27T01:17:42+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.7035423766667" lon="0.15566908962907178">
                    <time>2023-05-27T01:18:05+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>26</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.64479725910822" lon="0.14280708785811302">
                    <time>2023-05-27T01:22:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.64164062991751" lon="0.1353477686969554">
                    <time>2023-05-27T01:23:05+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.5653975831529" lon="0.11427956890965967">
                    <time>2023-05-27T01:27:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>30</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.5612182332133" lon="0.11978017841391721">
                    <time>2023-05-27T01:28:06+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>25</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.48945673065669" lon="0.166064213531155">
                    <time>2023-05-27T01:32:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.48289003527432" lon="0.16805992582240972">
                    <time>2023-05-27T01:33:07+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>23</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.40693057166275" lon="0.14513008983313652">
                    <time>2023-05-27T01:37:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>22</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.39770162614609" lon="0.1375522340181383">
                    <time>2023-05-27T01:38:16+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>19</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.304965171438745" lon="0.07798502403564961">
                    <time>2023-05-27T01:42:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>23</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.29778072985354" lon="0.06839212918091969">
                    <time>2023-05-27T01:43:17+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>24</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.20622706086127" lon="0.008905220690966664">
                    <time>2023-05-27T01:47:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>17</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.19734734846319" lon="0.0033468318824790458">
                    <time>2023-05-27T01:48:17+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>24</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.1035800687826" lon="-0.029087363899457014">
                    <time>2023-05-27T01:52:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>23</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.09698690926936" lon="-0.03939643946411675">
                    <time>2023-05-27T01:53:17+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>21</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.02933465204397" lon="-0.1404562405414243">
                    <time>2023-05-27T01:57:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="45.01903252842368" lon="-0.15343033078982024">
                    <time>2023-05-27T01:58:17+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.9417091502109" lon="-0.23098549124038867">
                    <time>2023-05-27T02:02:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.93626173202282" lon="-0.23199134272310323">
                    <time>2023-05-27T02:03:18+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.91212622878339" lon="-0.2370268675905434">
                    <time>2023-05-27T02:07:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.90869786347033" lon="-0.2384906321736237">
                    <time>2023-05-27T02:08:18+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.895191004035865" lon="-0.2677716405570462">
                    <time>2023-05-27T02:12:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>18</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.893205698048995" lon="-0.2708995827284391">
                    <time>2023-05-27T02:13:19+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.90400246415821" lon="-0.33528325280124055">
                    <time>2023-05-27T02:17:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.905055446026786" lon="-0.34117520531751827">
                    <time>2023-05-27T02:18:20+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.91525053675053" lon="-0.39740795221267045">
                    <time>2023-05-27T02:22:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>22</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.91594526222163" lon="-0.4014701852924877">
                    <time>2023-05-27T02:23:21+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>18</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.93124105067849" lon="-0.45748338058667704">
                    <time>2023-05-27T02:27:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>17</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.934986000519075" lon="-0.47095606044560556">
                    <time>2023-05-27T02:28:22+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>29</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.88265678070805" lon="-0.529423055266385">
                    <time>2023-05-27T02:32:43+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>36</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.88150314270445" lon="-0.5297165312729364">
                    <time>2023-05-27T02:33:32+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>38</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.87398238527758" lon="-0.5360812096273156">
                    <time>2023-05-27T02:37:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.85369373835359" lon="-0.5334844530009826">
                    <time>2023-05-27T02:42:39+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.846225047753016" lon="-0.5375164583545387">
                    <time>2023-05-27T02:43:23+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.827286745227966" lon="-0.5551142556190114">
                    <time>2023-05-27T02:47:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.82631810400523" lon="-0.5550577411686949">
                    <time>2023-05-27T02:57:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>16</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.82278721805849" lon="-0.5567660371250908">
                    <time>2023-05-27T02:57:57+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>24</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.783720720292365" lon="-0.5517085138167346">
                    <time>2023-05-27T03:02:50+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>21</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.781821014353895" lon="-0.551371560977062">
                    <time>2023-05-27T03:02:58+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>22</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.70819857506736" lon="-0.4919314843690629">
                    <time>2023-05-27T03:07:50+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>49</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.707589264879296" lon="-0.48759723645660946">
                    <time>2023-05-27T03:07:58+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>23</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.648231906918554" lon="-0.35762264616488854">
                    <time>2023-05-27T03:12:50+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>25</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.6461222115374" lon="-0.3549016955170966">
                    <time>2023-05-27T03:12:59+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>19</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.55392930423304" lon="-0.27162112374330144">
                    <time>2023-05-27T03:17:50+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>44</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.553208747695805" lon="-0.26351746812304644">
                    <time>2023-05-27T03:18:02+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>28</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.58465244079383" lon="-0.1205783263878408">
                    <time>2023-05-27T03:22:44+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.585822371711764" lon="-0.11102164620186968">
                    <time>2023-05-27T03:23:02+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>22</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.56566004427879" lon="0.03353775564161645">
                    <time>2023-05-27T03:27:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.56301294642705" lon="0.04475763590203413">
                    <time>2023-05-27T03:28:02+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.50646575064699" lon="0.16242986135645116">
                    <time>2023-05-27T03:32:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.50180710398678" lon="0.17055791051109695">
                    <time>2023-05-27T03:33:02+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.43072709787505" lon="0.2762046107164912">
                    <time>2023-05-27T03:37:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.42593393150946" lon="0.2869045915536181">
                    <time>2023-05-27T03:38:03+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.333464159171356" lon="0.3234520116604849">
                    <time>2023-05-27T03:42:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.32823665035485" lon="0.32890121561197005">
                    <time>2023-05-27T03:43:03+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.256886501888644" lon="0.3653643483967971">
                    <time>2023-05-27T03:47:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.25456317402962" lon="0.37498761313653883">
                    <time>2023-05-27T03:48:04+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.240805493991466" lon="0.48523164770943183">
                    <time>2023-05-27T03:52:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.24213221842225" lon="0.49763003158921404">
                    <time>2023-05-27T03:53:05+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.211221403044846" lon="0.6072233384624974">
                    <time>2023-05-27T03:57:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.208303870281746" lon="0.6111802881390368">
                    <time>2023-05-27T03:58:05+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>22</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.173555530740025" lon="0.6648478713010705">
                    <time>2023-05-27T04:02:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.1718199908453" lon="0.6764201163165762">
                    <time>2023-05-27T04:03:06+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.14456780618339" lon="0.7980991593259728">
                    <time>2023-05-27T04:07:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.14094883496196" lon="0.8085676497974394">
                    <time>2023-05-27T04:08:06+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.102724201376475" lon="0.9285604228201272">
                    <time>2023-05-27T04:12:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.10123957502143" lon="0.9414266161618392">
                    <time>2023-05-27T04:13:07+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.090636971465344" lon="1.0436807609120573">
                    <time>2023-05-27T04:17:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.09456301217011" lon="1.052906232902658">
                    <time>2023-05-27T04:18:07+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.06505587428266" lon="1.105242403219791">
                    <time>2023-05-27T04:22:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.0555024451103" lon="1.107524526911457">
                    <time>2023-05-27T04:23:07+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.032429989204694" lon="1.230865911817855">
                    <time>2023-05-27T04:27:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.02835004159217" lon="1.2450042593490376">
                    <time>2023-05-27T04:28:08+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.00933346770903" lon="1.3430075418497218">
                    <time>2023-05-27T04:32:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="44.004346169574376" lon="1.3402706604623975">
                    <time>2023-05-27T04:33:09+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="43.95568928830275" lon="1.2916218543437357">
                    <time>2023-05-27T04:37:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="43.9502659126231" lon="1.2858328629457227">
                    <time>2023-05-27T04:38:09+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="43.89754074424467" lon="1.2681725103201056">
                    <time>2023-05-27T04:42:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="43.89110497483868" lon="1.2706990641266107">
                    <time>2023-05-27T04:43:09+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>21</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="43.83233064943274" lon="1.2988024870656676">
                    <time>2023-05-27T04:47:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="43.82595823405947" lon="1.3028664561883416">
                    <time>2023-05-27T04:48:10+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="43.77262695914605" lon="1.3541684229552777">
                    <time>2023-05-27T04:52:49+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="43.7682155429467" lon="1.3574793338222104">
                    <time>2023-05-27T04:53:11+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="43.709378100378615" lon="1.3870930114418625">
                    <time>2023-05-27T04:57:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="43.702553665858034" lon="1.3907884891295645">
                    <time>2023-05-27T04:58:11+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="43.64548967056764" lon="1.4288558982817288">
                    <time>2023-05-27T05:02:50+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="43.6423785863955" lon="1.4314731063857293">
                    <time>2023-05-27T05:03:11+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="43.62264336186594" lon="1.4491005838581141">
                    <time>2023-05-27T05:07:50+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="43.62146944164873" lon="1.449931811360037">
                    <time>2023-05-27T05:08:11+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="43.61349484574218" lon="1.4537120228827507">
                    <time>2023-05-27T05:10:22+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>51</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="43.61339399943859" lon="1.4537474057500985">
                    <time>2023-05-27T05:12:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>15</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="43.61349484574218" lon="1.4537120228827507">
                    <time>2023-05-27T05:10:22+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>51</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                </trkseg>
              </trk>
            </gpx>""";

    private static final String GPX_DATA_ALL_POINTS_ON_ROUTED_PATHS = """
            <?xml version="1.0" encoding="UTF-8"?>
            <gpx xmlns="http://www.topografix.com/GPX/1/1" xmlns:gpxtpx="http://www.garmin.com/xmlschemas/TrackPointExtension/v1" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd" version="1.1" creator="gpx.py -- https://github.com/tkrajina/gpxpy">
              <trk>
                <trkseg>
                  <trkpt lat="48.8861387" lon="2.2568229">
                    <time>2024-08-13T07:11:22+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>972</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.8675681" lon="2.313585">
                    <time>2024-08-13T07:17:25+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>75</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.8644553" lon="2.3299827">
                    <time>2024-08-13T07:18:26+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>168</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.8624753" lon="2.3360331">
                    <time>2024-08-13T07:19:23+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>431</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.8627674" lon="2.3368234">
                    <time>2024-08-13T07:20:26+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>32</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                </trkseg>
              </trk>
            </gpx>
            """;

    private static final String GPX_Ligne3 = """
            <?xml version="1.0" encoding="UTF-8"?>
            <gpx xmlns="http://www.topografix.com/GPX/1/1" xmlns:gpxtpx="http://www.garmin.com/xmlschemas/TrackPointExtension/v1" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd" version="1.1" creator="gpx.py -- https://github.com/tkrajina/gpxpy">
              <trk>
                <trkseg>
                  <trkpt lat="48.873651728517984" lon="2.32728377836915">
                    <time>2024-07-09T11:22:36+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>810</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.87456665092364" lon="2.324190809183733">
                    <time>2024-07-09T11:24:20+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>810</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.87653477962411" lon="2.324384434372945">
                    <time>2024-07-09T11:26:26+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>810</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.87883973418161" lon="2.3214869087373415">
                    <time>2024-07-09T11:28:36+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>810</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                  <trkpt lat="48.88090251254795" lon="2.3149873711843725">
                    <time>2024-07-09T11:29:40+02:00</time>
                    <extensions>
                      <gpxtpx:accuracy>810</gpxtpx:accuracy>
                    </extensions>
                  </trkpt>
                </trkseg>
              </trk>
            </gpx>
            """;

    private static final String GPX_Paris_Cannes = readGPXFile("/home/laurent/IdeaProjects/graphhopper-with-rail/src/test/resources/GPX_Paris_Cannes.xml");

    /**
     * Read GPX file content as string
     */
    private static String readGPXFile(String filePath) {
        try {
            return Files.readString(Paths.get(filePath));
        } catch (IOException e) {
            throw new RuntimeException("Failed to read GPX file: " + filePath, e);
        }
    }

    /**
     * Test map matching with full GPX track data
     */
    @Test
    public void testFullGPXTrackMatching() {
        runGPXTrackMatchingTest(FULL_GPX_DATA, "FULL_GPX_DATA");
    }

    /**
     * Test map matching with GPX data where all points are on routed paths
     */
    @Test
    public void testGPXDataAllPointsOnRoutedPaths() {
        runGPXTrackMatchingTest(GPX_DATA_ALL_POINTS_ON_ROUTED_PATHS, "GPX_DATA_ALL_POINTS_ON_ROUTED_PATHS");
    }

    /**
     * Test map matching for metro line 3 trace
     */
    @Test
    public void testGPXDataLigne3() {
        runGPXTrackMatchingTest(GPX_Ligne3, "GPX_Ligne3");
    }

    /**
     * Test map matching for paris cannes
     */
    @Test
    public void testGPXDataParisCannes() {
        runGPXTrackMatchingTest(GPX_Paris_Cannes, "GPX_DATA_PARIS_CANNES");
    }

    /**
     * Common test method for GPX track matching
     */
    private void runGPXTrackMatchingTest(String gpxData, String testName) {
        DropwizardTestSupport<RailwayRoutingServerConfiguration> app = new DropwizardTestSupport<>(RailwayRoutingApplication.class, new File("config.yml").getAbsolutePath());
        String graphLocationProperty = "dw.graphhopper.graph.location";
        String previousGraphLocation = System.getProperty(graphLocationProperty);
        try {
            System.setProperty(graphLocationProperty, new File("./cache_gh").getAbsolutePath());
            app.before();
            int port = app.getLocalPort();
            HttpClient client = HttpClient.newHttpClient();
            HttpRequest request = HttpRequest.newBuilder().uri(URI.create("http://localhost:" + port + "/match?profile=all_tracks&type=json&max_visited_nodes=25000&max_processing_time=300&gps_accuracy=20&force_initial_routing=false&use_initial_routing=true&details=edge_key&traversal_keys=true")).header("Content-Type", "application/gpx+xml").POST(HttpRequest.BodyPublishers.ofString(gpxData, StandardCharsets.UTF_8)).build();

            HttpResponse<String> response = client.send(request, HttpResponse.BodyHandlers.ofString());
            assertEquals(200, response.statusCode(), "HTTP /match should succeed for " + testName + ". body=" + response.body());

            JsonNode json = new ObjectMapper().readTree(response.body());
            JsonNode paths = json.get("paths");
            assertNotNull(paths, "Response should include 'paths' for " + testName);
            assertTrue(paths.isArray() && paths.size() > 0, "Response should include at least one matched path for " + testName);
            assertTrue(paths.get(0).path("distance").asDouble() > 0, "Matched path distance should be > 0 for " + testName);

            JsonNode matching = json.get("map_matching");
            assertNotNull(matching, "Response should include 'map_matching' statistics for " + testName);
            assertTrue(matching.path("distance").asDouble() > 0, "Map matching distance should be > 0 for " + testName);
            // The usedDirectRouting field may not always be present, so we don't assert its presence

            // Check if observation_indexes are present
            JsonNode observationIndexes = json.get("observation_indexes");
            if (observationIndexes != null) {
                System.out.println("observation_indexes found for " + testName + ": " + observationIndexes.size() + " entries");
                // Validate that observation indexes are non-negative
                for (JsonNode idx : observationIndexes) {
                    if (idx.isArray() && idx.size() == 2) {
                        assertTrue(idx.get(0).asInt() >= 0, "First observation index should be >= 0 for " + testName);
                        assertTrue(idx.get(1).asInt() >= 0, "Second observation index should be >= 0 for " + testName);
                    }
                }
            } else {
                System.out.println("WARNING: No observation_indexes in response for " + testName);
            }
        } catch (Exception e) {
            fail("HTTP map matching should succeed with config.yml and profile=all_tracks for " + testName + ": " + e.getMessage());
        } finally {
            app.after();
            if (previousGraphLocation == null) {
                System.clearProperty(graphLocationProperty);
            } else {
                System.setProperty(graphLocationProperty, previousGraphLocation);
            }
        }
    }
}
