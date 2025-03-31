/*package frc.robot.commands.auto;

import org.json.simple.JSONObject;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.StringReader;
import javax.json.JsonReader;
import javax.json.Json;
import javax.json.JsonStructure;

import edu.wpi.first.wpilibj2.command.Command;

private class InvertIMUCommand extends Command{

        String filePath = "src/main/deploy/swerve";
        String jsonString;
        JSONObject jsonObject;
        JsonReader reader = Json.createReader(new FileReader("swervedrive.json"));
        

        public InvertIMUCommand(){
            String jsonString = new String(Files.readAllBytes(Paths.get(filePath)));
            JSONObject jsonObject = new JSONObject();
            jsonObject = jsonObject.getJsonObject(jsonString);
        }
            
        public void updateIMU(){   // Modify the JSON object
            jsonObject.put("invertedIMU", true);

            // Write the updated JSON back to the file
            String updatedJsonString = jsonObject.toString();

            FileWriter fileWriter = new FileWriter(filePath);
            fileWriter.write(updatedJsonString);
            fileWriter.close();
        }
}

//NOTE: SET IN ROBOT.JAVA
*/