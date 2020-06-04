using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//[ExecuteInEditMode]
public class KinematicSystem : MonoBehaviour
{
	/**The root of the kinematic system will be assigned in the inspector. 
	 * The rest of the bones and the tip will be determined at startup **/
	[SerializeField] KinematicBone root;
	[SerializeField] private KinematicBone tip;
	[SerializeField] private List<KinematicBone> bones;
	[SerializeField] Transform IKTarget;
	Dictionary<KinematicBone, Vector3> currentAngles;
	[SerializeField] float samplingDistance;
	[SerializeField] float learningRate;
	[SerializeField] float thresholdDistance; 

	/**This determines whether the system is updating Kinematics or Inverse Kinematics **/
	[SerializeField] bool inverse;
	private bool firstUpdateFlag; //This is necessary to link bones after all bones have been initialized

	// Start is called before the first frame update
	void Start()
	{
		firstUpdateFlag = true; 
	}

	void recursiveAdd(KinematicBone current)
	{
		bones.Add(current);
		Vector3 currentAngle = current.transform.rotation.eulerAngles * 1; 
		currentAngles.Add(current, currentAngle); 
		if(current.getChildren().Count != 0 && current != tip)
		{
			foreach(KinematicBone child in current.getChildren())
			{
				recursiveAdd(child); 
			}
		}
	}

	void firstUpdate()
	{
		bones = new List<KinematicBone>();
		currentAngles = new Dictionary<KinematicBone, Vector3>();
		recursiveAdd(root);
	}

	// Update is called once per frame
	void Update()
	{
		if(firstUpdateFlag)
		{
			firstUpdate();
			firstUpdateFlag = false; 
		}
		List<Vector3> angles = new List<Vector3>();
		foreach (KinematicBone bone in bones)
		{

			angles.Add(bone.getRelativeEulerAngles());
		}
		if (inverse)
		{
			
			updateInverseKinematics(IKTarget, angles);
		}
		else
		{
			updateKinematics(angles);
		}
	}

	public float distanceFromTarget(Transform target, List<Vector3> angles)
	{
		Vector3 tipLocation = updateKinematics(angles);
		return Vector3.Distance(tipLocation, target.position); 
	}

	public void updateInverseKinematics(Transform target, List<Vector3> angles)
	{
		for (int i = 0; i < bones.Count; i++)
		{
			float randomSign = -1;
			float randomExp = Random.Range(1, 10);
			randomSign = Mathf.Pow(randomSign, randomExp); 
			// Gradient descent - test each bone 3 times (for X, Y and Z directions)
			// X DIRECTION
				Vector3 savedAngle = angles[i];
				Vector3 currentAngle = savedAngle * 1; 
				float f_x = distanceFromTarget(target, angles);
				if (f_x > thresholdDistance)
				{
					currentAngle.x += samplingDistance * randomSign;
					angles[i] = currentAngle;
					float f_x_plus_d = distanceFromTarget(target, angles);
					float gradient = (f_x_plus_d - f_x) / (samplingDistance * randomSign);
					// Restores
					angles[i] = savedAngle;
					currentAngle = savedAngle;
					currentAngle.x -= (learningRate * gradient);
					angles[i] = currentAngle;
					//Y DIRECTION
					savedAngle = currentAngle;
					float f_y = distanceFromTarget(target, angles);
					currentAngle.y += samplingDistance * randomSign;
					angles[i] = currentAngle;
					float f_y_plus_d = distanceFromTarget(target, angles);
					gradient = (f_y_plus_d - f_y) / (samplingDistance * randomSign);
					// Restores
					angles[i] = savedAngle;
					currentAngle = savedAngle;
					currentAngle.y -= (learningRate * gradient);
					angles[i] = currentAngle;
					//Z DIRECTION
					savedAngle = currentAngle;
					float f_z = distanceFromTarget(target, angles);
					currentAngle.z += samplingDistance * randomSign;
					angles[i] = currentAngle;
					float f_z_plus_d = distanceFromTarget(target, angles);
					gradient = (f_z_plus_d - f_z) / (samplingDistance * randomSign);
					// Restores
					angles[i] = savedAngle;
					currentAngle = savedAngle;
					currentAngle.z -= (learningRate * gradient);
					//Debug.Log("Gradient Z = " + gradient); 
				}
			}
		updateKinematics(angles); 
	}

	Vector3 updateKinematics(List<Vector3> Angles)
	{
		if(Angles.Count != bones.Count)
		{
			Debug.Log("Angles array not the correct length - Bones count = " + bones.Count + ", Angles count = " + Angles.Count + "\n"); 
			return Vector3.zero; 
		}
		KinematicBone current;
		for (int i = 0; i < bones.Count; i++)
		{
			current = bones[i]; 
			//Debug.Log("Updating Kinematics for " + bones[i].name + "\n");
			current.updateKinematics(Angles[i]);
		}
		return tip.getEndPoint().transform.position; 
	}
}

