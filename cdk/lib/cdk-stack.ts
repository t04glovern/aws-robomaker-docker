import cdk = require('@aws-cdk/core');
import robomaker = require('@aws-cdk/aws-robomaker');
import iam = require('@aws-cdk/aws-iam');

export class CdkStack extends cdk.Stack {
  constructor(scope: cdk.Construct, id: string, props?: cdk.StackProps) {
    super(scope, id, props);

    const bucket_name = new cdk.CfnParameter(this, 'bucket-name-param', {
      type: 'String',
      default: 'devopstar'
    });

    const robo_tar_location = new cdk.CfnParameter(this, 'robo-tar-param', {
      type: 'String',
      default: 'resources/aws-robomaker-kickstart/input/simulation_ws.tar'
    })

    const robo_sim_app = new robomaker.CfnSimulationApplication(this, 'sim-app', {
      name: "aws-robomaker-kickstart",
      renderingEngine: {
        name: "OGRE",
        version: "1.x"
      },
      robotSoftwareSuite: {
        name: "ROS",
        version: "Melodic"
      },
      simulationSoftwareSuite: {
        name: "Gazebo",
        version: "9"
      },
      sources: [
        {
          architecture: "X86_64",
          s3Bucket: `${bucket_name.value}`,
          s3Key: `${robo_tar_location.value}`
        }
      ],
      tags: {
        project: "aws-robomaker-kickstart"
      }
    });

    const robo_sim_role = new iam.Role(this, "sim-role", {
      assumedBy: new iam.ServicePrincipal("robomaker.amazonaws.com")
    });

    robo_sim_role.addToPolicy(new iam.PolicyStatement({
      resources: [
        `arn:aws:s3:::${bucket_name.value}`
      ],
      actions: [
        "s3:ListBucket"
      ]
    }));

    robo_sim_role.addToPolicy(new iam.PolicyStatement({
      resources: [
        `arn:aws:s3:::${bucket_name.value}/*`
      ],
      actions: [
        "s3:Get*",
        "s3:List*",
        "s3:Put*",
        "s3:DeleteObject"
      ]
    }));

    robo_sim_role.addToPolicy(new iam.PolicyStatement({
      resources: [
        `arn:aws:logs:${cdk.Aws.REGION}:${cdk.Aws.ACCOUNT_ID}:log-group:/aws/robomaker/SimulationJobs*`
      ],
      actions: [
        "logs:CreateLogGroup",
        "logs:CreateLogStream",
        "logs:PutLogEvents",
        "logs:DescribeLogStreams"
      ]
    }));

    robo_sim_role.addToPolicy(new iam.PolicyStatement({
      resources: [
        `arn:aws:ec2:${cdk.Aws.REGION}:${cdk.Aws.ACCOUNT_ID}:*`
      ],
      actions: [
        "ec2:CreateNetworkInterfacePermission"
      ]
    }));

    robo_sim_role.addToPolicy(new iam.PolicyStatement({
      resources: [
        `*`
      ],
      actions: [
        "cloudwatch:PutMetricData",
        "ec2:AssociateRouteTable",
        "ec2:CreateSubnet",
        "ec2:DeleteNetworkInterface",
        "ec2:DeleteSubnet",
        "ec2:DescribeNetworkInterfaces",
        "ec2:DescribeSecurityGroups",
        "ec2:DescribeSubnets",
        "ec2:DescribeVpcs"
      ]
    }));

    new cdk.CfnOutput(this, 'robo-sim-role-output', {
      value: robo_sim_role.roleArn,
    });

    new cdk.CfnOutput(this, 'robo-sim-output', {
      value: robo_sim_app.attrArn,
    });

  }
}
